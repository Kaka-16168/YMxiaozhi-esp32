#!/usr/bin/env python3
"""Xiaozhi Assistant build script."""

import sys
import os
import json
import zipfile
import argparse
import re
import subprocess
from pathlib import Path
from typing import Any, Optional

# Switch to project root directory
os.chdir(Path(__file__).resolve().parent.parent)

_DEFAULT_IDF_VERSION = (6, 0, 2)
_DEFAULT_LANGUAGE = "en-US"

_LITE_WAKE_WORD_TARGETS = {"esp32c3", "esp32c5", "esp32c6"}
_AFE_WAKE_WORD_TARGETS = {"esp32s3", "esp32p4", "esp32s31"}
_ESP_WAKE_WORD_TARGETS = {"esp32", *_LITE_WAKE_WORD_TARGETS}
_WAKE_WORD_TARGETS = (
    "esp32",
    "esp32c3",
    "esp32c5",
    "esp32c6",
    "esp32s3",
    "esp32p4",
    "esp32s31",
)
_WAKE_WORD_MODEL_PATTERN = re.compile(r"^wn9[sl]?_[a-z0-9_]+$")
_ESP_SR_KCONFIG = Path(
    "managed_components/espressif__esp-sr/Kconfig.projbuild"
)


def _emit_build_stage(stage: str) -> None:
    """Emit a machine-readable stage marker for cloud build runners."""
    enabled = os.environ.get("XIAOZHI_BUILD_STAGES", "").strip().casefold()
    if enabled in {"1", "true", "yes", "on"}:
        print(f"XIAOZHI_STAGE {stage}", flush=True)


def get_project_version() -> Optional[str]:
    """Read set(PROJECT_VER "x.y.z") from root CMakeLists.txt"""
    with Path("CMakeLists.txt").open(encoding='utf-8') as f:
        for line in f:
            if line.startswith("set(PROJECT_VER"):
                return line.split("\"")[1]
    return None


def _get_idf_command() -> list[str]:
    """Get the command used to invoke the active ESP-IDF."""
    idf_path = os.environ.get("IDF_PATH")

    if idf_path:
        idf_py = Path(idf_path) / "tools" / "idf.py"
        if idf_py.is_file():
            return [sys.executable, str(idf_py)]

    raise RuntimeError(
        "ESP-IDF environment is not initialized correctly. "
        "IDF_PATH/tools/idf.py was not found."
    )


def _run_idf(*args: str, preview: bool = False) -> None:
    command = _get_idf_command()
    if preview:
        command.append("--preview")
    command.extend(args)
    if subprocess.run(command, check=False).returncode != 0:
        print(f"{' '.join(command)} failed", file=sys.stderr)
        sys.exit(1)


def merge_bin(preview: bool = False) -> None:
    _run_idf("merge-bin", preview=preview)


def zip_bin(name: str, version: str) -> None:
    """Zip build/merged-binary.bin to releases/v{version}_{name}.zip"""
    out_dir = Path("releases")
    out_dir.mkdir(exist_ok=True)
    output_path = out_dir / f"v{version}_{name}.zip"

    if output_path.exists():
        output_path.unlink()

    with zipfile.ZipFile(output_path, "w", compression=zipfile.ZIP_DEFLATED) as zipf:
        zipf.write("build/merged-binary.bin", arcname="merged-binary.bin")
    print(f"zip bin to {output_path} done")


def _get_manufacturer(cfg: dict) -> Optional[str]:
    """Read manufacturer from config.json"""
    m = cfg.get("manufacturer")
    if isinstance(m, str) and m.strip():
        return m.strip()
    return None


_REPORTED_IDENTIFIER_PATTERN = re.compile(r"^[a-z0-9.-]+$")


def _validate_reported_identifier(value: object, field: str) -> str:
    """Validate an OTA-reported board type or name."""
    if not isinstance(value, str) or not value:
        raise ValueError(f"missing non-empty {field}")
    if not _REPORTED_IDENTIFIER_PATTERN.fullmatch(value):
        raise ValueError(
            f"{field} {value!r} must contain only lowercase letters, "
            'digits, "." and "-"'
        )
    return value


def _get_reported_type(cfg: dict) -> str:
    """Read the compatibility-sensitive board type from config.json."""
    return _validate_reported_identifier(cfg.get("type"), 'top-level "type"')


def _get_reported_name(build: dict) -> str:
    """Read the compatibility-sensitive board name from a build entry."""
    return _validate_reported_identifier(build.get("name"), 'build "name"')


def _get_full_name(manufacturer: Optional[str], name: str) -> str:
    """Return the artifact name without duplicating an existing manufacturer prefix."""
    prefix = f"{manufacturer}-" if manufacturer else ""
    return name if not prefix or name.startswith(prefix) else f"{prefix}{name}"


def _normalize_p4x_release_name(name: str) -> str:
    """Represent P4X as a chip-family segment instead of a trailing suffix."""
    if name.endswith("-p4x"):
        name_without_suffix = name[:-4]
        if "-p4-" in name_without_suffix:
            return name_without_suffix.replace("-p4-", "-p4x-", 1)
    return name


def _get_release_full_name(
    manufacturer: Optional[str],
    build: dict,
) -> str:
    """Return manufacturer + board name for the release artifact."""
    release_name = _get_reported_name(build)
    release_name = _normalize_p4x_release_name(release_name)
    return _get_full_name(manufacturer, release_name)


def _normalize_language(language: str) -> str:
    """Normalize a locale accepted by the build interface."""
    supported_languages = _collect_languages()
    languages_by_casefold = {
        supported.casefold(): supported
        for supported in supported_languages
    }
    normalized = language.strip().replace("_", "-").casefold()
    try:
        return languages_by_casefold[normalized]
    except KeyError as error:
        supported = ", ".join(supported_languages)
        raise ValueError(
            f"Unsupported language {language!r}. Supported values: {supported}"
        ) from error


def _collect_languages(
    cmake_path: Path = Path("main/CMakeLists.txt"),
    kconfig_path: Path = Path("main/Kconfig.projbuild"),
    locales_dir: Path = Path("main/assets/locales"),
) -> list[str]:
    """Read configured locale mappings and validate their build resources."""
    for path in (cmake_path, kconfig_path, locales_dir):
        if not path.exists():
            raise RuntimeError(f"Language configuration source not found: {path}")

    cmake = cmake_path.read_text(encoding="utf-8")
    mappings = re.findall(
        r"(?:if|elseif)\(CONFIG_LANGUAGE_([A-Z_]+)\)\s*"
        r'set\(LANG_DIR "([^"]+)"\)',
        cmake,
    )
    if not mappings:
        raise RuntimeError(f"No language mappings found in {cmake_path}")

    kconfig = kconfig_path.read_text(encoding="utf-8")
    errors: list[str] = []
    languages: list[str] = []
    for symbol, language in mappings:
        if not re.search(
            rf"^\s*config LANGUAGE_{re.escape(symbol)}$",
            kconfig,
            re.MULTILINE,
        ):
            errors.append(f"CONFIG_LANGUAGE_{symbol} is missing from {kconfig_path}")
        if not (locales_dir / language).is_dir():
            errors.append(f"locale directory is missing: {locales_dir / language}")
        if language in languages:
            errors.append(f"duplicate language mapping: {language}")
        else:
            languages.append(language)

    if errors:
        details = "\n".join(f"  - {error}" for error in errors)
        raise RuntimeError(f"Invalid language configuration:\n{details}")
    return languages


def _language_sdkconfig_option(language: str) -> tuple[str, str]:
    """Return the normalized locale and its Kconfig assignment."""
    normalized = _normalize_language(language)
    symbol = normalized.replace("-", "_").upper()
    return normalized, f"CONFIG_LANGUAGE_{symbol}=y"


def _collect_wake_words(
    kconfig_path: Path = _ESP_SR_KCONFIG,
) -> list[dict[str, object]]:
    """Read available WakeNet models from the resolved ESP-SR component."""
    if not kconfig_path.exists():
        raise RuntimeError(
            f"{kconfig_path} was not found. Resolve managed components with "
            "'idf.py reconfigure' before listing wake words."
        )

    content = kconfig_path.read_text(encoding="utf-8")
    config_matches = list(re.finditer(
        r"^\s*config SR_WN_([A-Z0-9_]+)\s*$",
        content,
        re.MULTILINE,
    ))
    wake_words: list[dict[str, object]] = []
    for index, match in enumerate(config_matches):
        block_end = (
            config_matches[index + 1].start()
            if index + 1 < len(config_matches)
            else len(content)
        )
        block = content[match.end():block_end]
        label_match = re.search(r'^\s*bool "([^"]+)"\s*$', block, re.MULTILINE)
        if not label_match:
            continue

        model = match.group(1).lower()
        label = label_match.group(1)
        suffix = f"({model})"
        phrase = (
            label[:-len(suffix)].strip()
            if label.endswith(suffix)
            else label
        )
        targets = (
            _WAKE_WORD_TARGETS
            if model.startswith("wn9s_")
            else (
                "esp32",
                "esp32s3",
                "esp32p4",
                "esp32s31",
            )
        )
        wake_words.append({
            "model": model,
            "phrase": phrase,
            "targets": list(targets),
        })

    if not wake_words:
        raise RuntimeError(f"No WakeNet models found in {kconfig_path}")
    return wake_words


def _print_wake_word_list(wake_words: list[dict[str, object]]) -> None:
    """Print a human-readable table of available wake-word models."""
    model_width = max(
        len("MODEL"),
        *(len(str(item["model"])) for item in wake_words),
    )
    phrase_width = max(
        len("PHRASE"),
        *(len(str(item["phrase"])) for item in wake_words),
    )
    print(f"{'MODEL':<{model_width}}  {'PHRASE':<{phrase_width}}  TARGETS")
    for item in wake_words:
        targets = ",".join(str(target) for target in item["targets"])
        print(
            f"{str(item['model']):<{model_width}}  "
            f"{str(item['phrase']):<{phrase_width}}  {targets}"
        )
    print("\nSpecial values: nihaoxiaozhi, hiesp, disabled")


def _board_supports_wake_word(
    target: str,
    sdkconfig_append: list[str],
) -> bool:
    """Respect board opt-outs as well as target and PSRAM requirements."""
    if _sdkconfig_assignments(sdkconfig_append).get("CONFIG_WAKE_WORD_DISABLED") == "y":
        return False
    if target in _LITE_WAKE_WORD_TARGETS:
        return True
    if target not in _AFE_WAKE_WORD_TARGETS | {"esp32"}:
        return False

    defaults: list[str] = []
    for path in (Path("sdkconfig.defaults"), Path(f"sdkconfig.defaults.{target}")):
        if not path.exists():
            continue
        defaults.extend(
            line.strip()
            for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip().startswith("CONFIG_") and "=" in line
        )
    assignments = _sdkconfig_assignments(
        _merge_sdkconfig_options(defaults, sdkconfig_append)
    )
    return assignments.get("CONFIG_SPIRAM") == "y"


def _wake_word_sdkconfig_options(
    wake_word: str,
    target: str,
) -> tuple[str, list[str], list[str]]:
    """Map a wake-word model to implementation and model Kconfig options."""
    normalized = wake_word.strip().casefold().replace("-", "_")
    if normalized == "nihaoxiaozhi":
        normalized = (
            "wn9s_nihaoxiaozhi"
            if target in _LITE_WAKE_WORD_TARGETS
            else "wn9_nihaoxiaozhi_tts"
        )
    elif normalized == "hiesp":
        normalized = "wn9s_hiesp" if target in _LITE_WAKE_WORD_TARGETS else "wn9_hiesp"

    options = [
        "CONFIG_WAKE_WORD_DISABLED=n",
        "CONFIG_USE_ESP_WAKE_WORD=n",
        "CONFIG_USE_AFE_WAKE_WORD=n",
        "CONFIG_USE_CUSTOM_WAKE_WORD=n",
    ]

    if normalized == "disabled":
        options.append("CONFIG_WAKE_WORD_DISABLED=y")
        return normalized, options, ["CONFIG_WAKE_WORD_DISABLED"]

    if target not in _ESP_WAKE_WORD_TARGETS | _AFE_WAKE_WORD_TARGETS:
        raise ValueError(f"Wake-word selection is not supported for target {target}")
    if not _WAKE_WORD_MODEL_PATTERN.fullmatch(normalized):
        raise ValueError(
            f"Invalid wake word {wake_word!r}. Use 'disabled', "
            "'nihaoxiaozhi', 'hiesp', or an ESP-SR model name such as "
            "'wn9_jarvis_tts'."
        )
    if target in _LITE_WAKE_WORD_TARGETS and not normalized.startswith("wn9s_"):
        raise ValueError(
            f"Target {target} supports WakeNet9s models only; "
            f"{normalized!r} is not compatible"
        )

    implementation = (
        "CONFIG_USE_AFE_WAKE_WORD"
        if target in _AFE_WAKE_WORD_TARGETS
        else "CONFIG_USE_ESP_WAKE_WORD"
    )
    model_symbol = f"CONFIG_SR_WN_{normalized.upper()}"
    options.extend((f"{implementation}=y", f"{model_symbol}=y"))
    return normalized, options, [implementation, model_symbol]


################################################################################
# board / variant related functions
################################################################################

_BOARDS_DIR = Path("main/boards")

_DISPLAY_STYLE_SYMBOLS = {
    "default": "CONFIG_USE_DEFAULT_MESSAGE_STYLE",
    "wechat": "CONFIG_USE_WECHAT_MESSAGE_STYLE",
    "emote": "CONFIG_USE_EMOTE_MESSAGE_STYLE",
}
_DYNAMIC_CAMERA_MIRROR_BOARD_CONFIGS = {
    "CONFIG_BOARD_TYPE_DF_S3_AI_CAM",
    "CONFIG_BOARD_TYPE_ESP_SPARKBOT",
    "CONFIG_BOARD_TYPE_M5STACK_ATOM_S3R_CAM_M12_ECHO_BASE",
    "CONFIG_BOARD_TYPE_SEEED_STUDIO_SENSECAP_WATCHER",
}
_AUTO_CAMERA_ORIENTATION_BOARD_CONFIGS = {
    "CONFIG_BOARD_TYPE_ESP32_S31_KORVO_1",
}
_OPTIONAL_CAMERA_ENABLE_SYMBOLS = {
    "CONFIG_BOARD_TYPE_ESP_VOCAT": "CONFIG_ESP_VIDEO_ENABLE_USB_UVC_VIDEO_DEVICE",
    "CONFIG_BOARD_TYPE_CYBERAI_VOCAT_V2_0": "CONFIG_ESP_VIDEO_ENABLE_USB_UVC_VIDEO_DEVICE",
}
_COMMON_CAMERA_CONSTRUCTOR_RE = re.compile(
    r"\bnew(?:\s*\(\s*std::nothrow\s*\))?\s+Esp(?:32Camera|Video)\b"
)


def _sdkconfig_assignments(options: list[str]) -> dict[str, str]:
    """Return the final value for each CONFIG_* assignment in a fragment."""
    assignments: dict[str, str] = {}
    for option in options:
        key, separator, value = option.strip().partition("=")
        if not separator or not key.startswith("CONFIG_"):
            raise ValueError(f"Invalid sdkconfig assignment: {option!r}")
        assignments[key] = value
    return assignments


def _kconfig_choice(
    name: str,
    kconfig_path: Path = Path("main/Kconfig.projbuild"),
) -> dict[str, Any]:
    """Read a named project choice without trying to reimplement Kconfig."""
    content = kconfig_path.read_text(encoding="utf-8")
    start = re.search(rf"^choice\s+{re.escape(name)}\s*$", content, re.MULTILINE)
    if not start:
        raise ValueError(f"Kconfig choice {name} was not found in {kconfig_path}")
    end = re.search(r"^endchoice\s*$", content[start.end():], re.MULTILINE)
    if not end:
        raise ValueError(f"Kconfig choice {name} has no endchoice")
    block = content[start.end():start.end() + end.start()]
    entries: list[dict[str, str]] = []
    matches = list(re.finditer(r"^\s*config\s+([A-Za-z0-9_]+)\s*$", block, re.MULTILINE))
    for index, match in enumerate(matches):
        entry_end = matches[index + 1].start() if index + 1 < len(matches) else len(block)
        entry = block[match.end():entry_end]
        label = re.search(r'^\s*bool\s+"([^"]+)"', entry, re.MULTILINE)
        if label:
            entries.append({"value": match.group(1), "label": label.group(1)})
    default = re.search(r"^\s*default\s+([A-Za-z0-9_]+)", block, re.MULTILINE)
    return {
        "entries": entries,
        "default": default.group(1) if default else None,
        "board_configs": {
            f"CONFIG_{symbol}"
            for symbol in re.findall(r"\b(BOARD_TYPE_[A-Za-z0-9_]+)\b", block)
        },
        "block": block,
    }


def _kconfig_config_board_dependencies(
    symbol: str,
    kconfig_path: Path = Path("main/Kconfig.projbuild"),
) -> set[str]:
    """Return CONFIG_BOARD_TYPE_* names mentioned by one config entry."""
    content = kconfig_path.read_text(encoding="utf-8")
    start = re.search(rf"^\s*config\s+{re.escape(symbol)}\s*$", content, re.MULTILINE)
    if not start:
        raise ValueError(f"Kconfig config {symbol} was not found in {kconfig_path}")
    remainder = content[start.end():]
    end = re.search(r"^\s*(?:config\s+|endchoice\s*$|endmenu\s*$)", remainder, re.MULTILINE)
    block = remainder[:end.start()] if end else remainder
    return {
        f"CONFIG_{item}"
        for item in re.findall(r"\b(BOARD_TYPE_[A-Za-z0-9_]+)\b", block)
    }


def _board_source_text(board: str) -> str:
    board_dir = _BOARDS_DIR / board
    parts: list[str] = []
    for path in sorted(board_dir.rglob("*")):
        if path.suffix in {".c", ".cc", ".cpp", ".h", ".hpp"}:
            parts.append(path.read_text(encoding="utf-8", errors="replace"))
    return "\n".join(parts)


def _selected_choice_default(
    choice: dict[str, Any],
    assignments: dict[str, str],
) -> str:
    for entry in choice["entries"]:
        if assignments.get(f"CONFIG_{entry['value']}") == "y":
            return str(entry["value"])
    default = choice.get("default")
    if not default:
        raise ValueError("Exposed Kconfig choice has no default")
    return str(default)


def _build_option_definitions(
    board: str,
    target: str,
    board_config: str,
    build: dict[str, Any],
) -> list[dict[str, Any]]:
    """Describe the curated, board-compatible options exposed to callers."""
    sdkconfig_append = build.get("sdkconfig_append", [])
    if not isinstance(sdkconfig_append, list) or not all(isinstance(item, str) for item in sdkconfig_append):
        raise ValueError(f"build {build.get('name')!r} sdkconfig_append must be a string list")
    assignments = _sdkconfig_assignments(sdkconfig_append)
    source = _board_source_text(board)
    display_symbols = set(re.findall(r"\bCONFIG_(?:OLED|LCD)_[A-Z0-9_]+\b", source))
    definitions: list[dict[str, Any]] = []

    network_choice = _kconfig_choice("XIAOZHI_NETWORK_TYPE")
    if board_config in network_choice["board_configs"]:
        definitions.append({
            "key": "network_type",
            "type": "select",
            "default": (
                "ethernet"
                if assignments.get("CONFIG_XIAOZHI_NETWORK_ETHERNET") == "y"
                else "wifi"
            ),
            "choices": [
                {"value": "wifi", "label": "Wi-Fi"},
                {"value": "ethernet", "label": "Ethernet"},
            ],
        })

    for choice_name in ("DISPLAY_OLED_TYPE", "DISPLAY_LCD_TYPE"):
        choice = _kconfig_choice(choice_name)
        if board_config not in choice["board_configs"]:
            continue
        entries = [
            entry for entry in choice["entries"]
            if entry["value"] != "LCD_CUSTOM"
            and f"CONFIG_{entry['value']}" in display_symbols
        ]
        if not entries:
            continue
        definitions.append({
            "key": "display_model",
            "type": "select",
            "default": _selected_choice_default(choice, assignments),
            "choices": entries,
        })
        break

    if re.search(r"\b[A-Za-z0-9_]*LcdDisplay\b", source):
        style_choice = _kconfig_choice("DISPLAY_STYLE")
        emote_boards = _kconfig_config_board_dependencies("USE_EMOTE_MESSAGE_STYLE")
        style_choices = [
            {"value": "default", "label": "Default"},
            {"value": "wechat", "label": "WeChat"},
        ]
        if board_config in emote_boards:
            style_choices.append({"value": "emote", "label": "Emote animation"})
        style_default = "default"
        selected_style = _selected_choice_default(style_choice, assignments)
        for value, symbol in _DISPLAY_STYLE_SYMBOLS.items():
            if symbol == f"CONFIG_{selected_style}":
                style_default = value
                break
        definitions.extend((
            {
                "key": "display_style",
                "type": "select",
                "default": style_default,
                "choices": style_choices,
            },
            {
                "key": "multiline_chat",
                "type": "boolean",
                "default": assignments.get("CONFIG_USE_MULTILINE_CHAT_MESSAGE") == "y",
            },
        ))

    aec_boards = _kconfig_config_board_dependencies("USE_DEVICE_AEC")
    if board_config in aec_boards:
        definitions.append({
            "key": "aec_mode",
            "type": "select",
            "default": "device" if assignments.get("CONFIG_USE_DEVICE_AEC") == "y" else "off",
            "choices": [
                {"value": "off", "label": "Off"},
                {"value": "device", "label": "Device-side AEC"},
            ],
        })

    if target != "esp32p4" and ("wifi_board.h" in source or re.search(r"\bWifiBoard\b", source)):
        definitions.append({
            "key": "wifi_provisioning",
            "type": "select",
            "default": (
                "blufi"
                if assignments.get("CONFIG_USE_ESP_BLUFI_WIFI_PROVISIONING") == "y"
                and assignments.get("CONFIG_USE_HOTSPOT_WIFI_PROVISIONING") == "n"
                else "hotspot"
            ),
            "choices": [
                {"value": "hotspot", "label": "Wi-Fi hotspot"},
                {"value": "blufi", "label": "ESP-BluFi"},
            ],
        })

    camera_enable_symbol = _OPTIONAL_CAMERA_ENABLE_SYMBOLS.get(board_config)
    has_common_camera = (
        _COMMON_CAMERA_CONSTRUCTOR_RE.search(source) is not None
        and (
            camera_enable_symbol is None
            or assignments.get(camera_enable_symbol) == "y"
        )
    )
    if has_common_camera and board_config not in _DYNAMIC_CAMERA_MIRROR_BOARD_CONFIGS:
        if board_config in _AUTO_CAMERA_ORIENTATION_BOARD_CONFIGS:
            definitions.append({
                "key": "camera_orientation",
                "type": "select",
                "default": "auto",
                "choices": [
                    {"value": "auto", "label": "Sensor default"},
                    {"value": "manual", "label": "Manual"},
                ],
            })
        definitions.extend((
            {"key": "camera_hmirror", "type": "boolean", "default": False},
            {"key": "camera_vflip", "type": "boolean", "default": False},
        ))

    configured_defaults = build.get("build_options", {})
    if not isinstance(configured_defaults, dict):
        raise ValueError(f"build {build.get('name')!r} build_options must be an object")
    by_key = {definition["key"]: definition for definition in definitions}
    unknown_defaults = sorted(set(configured_defaults) - set(by_key))
    if unknown_defaults:
        raise ValueError(
            f"build {build.get('name')!r} has unsupported build_options defaults: "
            + ", ".join(unknown_defaults)
        )
    for key, value in configured_defaults.items():
        by_key[key]["default"] = value

    _normalize_build_options(definitions, {})
    return definitions


def _normalize_build_options(
    definitions: list[dict[str, Any]],
    requested: object,
) -> dict[str, object]:
    if not isinstance(requested, dict):
        raise ValueError("--build-options-json must contain a JSON object")
    by_key = {definition["key"]: definition for definition in definitions}
    unknown = sorted(set(requested) - set(by_key))
    if unknown:
        raise ValueError("Unsupported build option(s): " + ", ".join(unknown))

    normalized: dict[str, object] = {}
    for key in sorted(by_key):
        definition = by_key[key]
        value = requested.get(key, definition["default"])
        if definition["type"] == "boolean":
            if not isinstance(value, bool):
                raise ValueError(f"Build option {key} must be a boolean")
        elif definition["type"] == "select":
            allowed = {choice["value"] for choice in definition["choices"]}
            if not isinstance(value, str) or value not in allowed:
                raise ValueError(
                    f"Build option {key} must be one of: {', '.join(sorted(allowed))}"
                )
        normalized[key] = value

    if (
        "camera_orientation" in by_key
        and "camera_orientation" not in requested
        and {"camera_hmirror", "camera_vflip"}.intersection(requested)
    ):
        normalized["camera_orientation"] = "manual"

    if normalized.get("display_style") != "default" and "multiline_chat" in normalized:
        normalized["multiline_chat"] = False
    return normalized


def _build_options_sdkconfig(
    definitions: list[dict[str, Any]],
    options: dict[str, object],
    base_assignments: dict[str, str],
) -> list[str]:
    """Expand semantic build options into a complete, mutually-exclusive fragment."""
    by_key = {definition["key"]: definition for definition in definitions}
    result: list[str] = []

    if "network_type" in options:
        ethernet = options["network_type"] == "ethernet"
        result.extend((
            f"CONFIG_XIAOZHI_NETWORK_WIFI={'n' if ethernet else 'y'}",
            f"CONFIG_XIAOZHI_NETWORK_ETHERNET={'y' if ethernet else 'n'}",
        ))

    if "display_model" in options:
        selected = options["display_model"]
        choice_name = (
            "DISPLAY_LCD_TYPE" if str(selected).startswith("LCD_") else "DISPLAY_OLED_TYPE"
        )
        for choice in _kconfig_choice(choice_name)["entries"]:
            result.append(f"CONFIG_{choice['value']}={'y' if choice['value'] == selected else 'n'}")

    if "display_style" in options:
        selected = options["display_style"]
        for choice in by_key["display_style"]["choices"]:
            value = choice["value"]
            symbol = _DISPLAY_STYLE_SYMBOLS[value]
            result.append(f"{symbol}={'y' if value == selected else 'n'}")
        flash_symbols = (
            "CONFIG_FLASH_NONE_ASSETS",
            "CONFIG_FLASH_DEFAULT_ASSETS",
            "CONFIG_FLASH_CUSTOM_ASSETS",
            "CONFIG_FLASH_EXPRESSION_ASSETS",
        )
        if selected == "emote" and base_assignments.get("CONFIG_FLASH_CUSTOM_ASSETS") != "y":
            result.extend(
                f"{symbol}={'y' if symbol == 'CONFIG_FLASH_EXPRESSION_ASSETS' else 'n'}"
                for symbol in flash_symbols
            )
        elif selected != "emote" and base_assignments.get("CONFIG_FLASH_EXPRESSION_ASSETS") == "y":
            result.extend(
                f"{symbol}={'y' if symbol == 'CONFIG_FLASH_DEFAULT_ASSETS' else 'n'}"
                for symbol in flash_symbols
            )

    if "multiline_chat" in options:
        result.append(f"CONFIG_USE_MULTILINE_CHAT_MESSAGE={'y' if options['multiline_chat'] else 'n'}")

    if "aec_mode" in options:
        device = options["aec_mode"] == "device"
        result.extend((
            f"CONFIG_USE_DEVICE_AEC={'y' if device else 'n'}",
            "CONFIG_USE_SERVER_AEC=n",
        ))
        if device:
            result.append("CONFIG_USE_AUDIO_PROCESSOR=y")

    if "wifi_provisioning" in options:
        blufi = options["wifi_provisioning"] == "blufi"
        result.extend((
            f"CONFIG_USE_HOTSPOT_WIFI_PROVISIONING={'n' if blufi else 'y'}",
            f"CONFIG_USE_ESP_BLUFI_WIFI_PROVISIONING={'y' if blufi else 'n'}",
        ))

    if options.get("camera_orientation") == "auto":
        result.append("CONFIG_XIAOZHI_CAMERA_MIRROR_CONFIGURED=n")
    elif "camera_hmirror" in options or "camera_vflip" in options:
        result.extend((
            "CONFIG_XIAOZHI_CAMERA_MIRROR_CONFIGURED=y",
            f"CONFIG_XIAOZHI_CAMERA_HMIRROR={'y' if options.get('camera_hmirror') else 'n'}",
            f"CONFIG_XIAOZHI_CAMERA_VFLIP={'y' if options.get('camera_vflip') else 'n'}",
        ))
    return result


def _parse_version(value: str) -> tuple[int, int, int]:
    """Parse an ESP-IDF version string such as 5.5.4 or v6.0."""
    match = re.search(r"v?(\d+)\.(\d+)(?:\.(\d+))?", value)
    if not match:
        raise ValueError(f"Invalid ESP-IDF version: {value}")
    return tuple(int(part or 0) for part in match.groups())


def _detect_idf_version() -> tuple[int, int, int]:
    """Resolve the active ESP-IDF version for version-gated build variants."""
    idf_path = os.environ.get("IDF_PATH")
    if idf_path:
        version_file = Path(idf_path) / "tools/cmake/version.cmake"
        if version_file.exists():
            values: dict[str, int] = {}
            for line in version_file.read_text(encoding="utf-8").splitlines():
                match = re.match(r"set\(IDF_VERSION_(MAJOR|MINOR|PATCH)\s+(\d+)\)", line)
                if match:
                    values[match.group(1)] = int(match.group(2))
            if all(part in values for part in ("MAJOR", "MINOR", "PATCH")):
                return values["MAJOR"], values["MINOR"], values["PATCH"]

    try:
        output = subprocess.run(
            ["idf.py", "--version"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
        return _parse_version(output)
    except (FileNotFoundError, subprocess.CalledProcessError, ValueError) as error:
        raise RuntimeError(
            "ESP-IDF version was not detected. Source export.sh before running build.py."
        ) from error


def _detect_idf_version_for_listing() -> tuple[int, int, int]:
    """Detect IDF when available, otherwise list the preferred IDF 6.0 variants."""
    try:
        return _detect_idf_version()
    except RuntimeError:
        version = ".".join(str(part) for part in _DEFAULT_IDF_VERSION)
        print(
            f"[WARN] ESP-IDF is not active; listing variants for ESP-IDF {version}.",
            file=sys.stderr,
        )
        return _DEFAULT_IDF_VERSION


def _version_matches(version: tuple[int, int, int], expression: str) -> bool:
    """Evaluate a single comparison such as '<6.0' or '>=6.0.1'."""
    match = re.fullmatch(r"\s*(<=|>=|<|>|==)\s*(v?\d+\.\d+(?:\.\d+)?)\s*", expression)
    if not match:
        raise ValueError(f"Invalid ESP-IDF version expression: {expression}")
    operator, expected_text = match.groups()
    expected = _parse_version(expected_text)
    return {
        "<": version < expected,
        "<=": version <= expected,
        ">": version > expected,
        ">=": version >= expected,
        "==": version == expected,
    }[operator]


def _get_builds_for_idf(cfg: dict, idf_version: tuple[int, int, int]) -> list[dict]:
    """Return build entries whose optional ESP-IDF version rule matches."""
    builds: list[dict] = []
    for build in cfg.get("builds", []):
        _get_reported_name(build)
        expression = build.get("idf_version")
        if expression and not _version_matches(idf_version, expression):
            continue
        builds.append(dict(build))
    return builds


def _get_board_display_name(
    config_symbol: str,
    kconfig_path: Path = Path("main/Kconfig.projbuild"),
) -> str:
    """Return the user-facing bool prompt for a CONFIG_BOARD_TYPE_* symbol."""
    symbol = config_symbol.removeprefix("CONFIG_")
    if not kconfig_path.exists():
        raise ValueError(f"Board Kconfig file not found: {kconfig_path}")

    in_symbol = False
    for line in kconfig_path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped.startswith("config "):
            if in_symbol:
                break
            in_symbol = stripped.split("config ", 1)[1].strip() == symbol
            continue
        if not in_symbol:
            continue
        prompt = re.match(r'^bool\s+"([^"]+)"', stripped)
        if prompt:
            return prompt.group(1)
        if stripped.startswith(("choice ", "endchoice", "menu ", "endmenu")):
            break

    raise ValueError(
        f'Kconfig bool prompt not found for {config_symbol} in {kconfig_path}'
    )


def _collect_variants(
    config_filename: str = "config.json",
    idf_version: tuple[int, int, int] = (6, 0, 0),
) -> list[dict[str, Any]]:
    """Traverse all boards under main/boards, collect variant information."""
    variants: list[dict[str, str]] = []
    errors: list[str] = []
    type_owners: dict[str, str] = {}
    name_owners: dict[str, str] = {}
    identity_owners: dict[tuple[str, str], str] = {}

    for cfg_path in sorted(_BOARDS_DIR.rglob(config_filename)):
        board_dir = cfg_path.parent
        if board_dir.name == "common":
            continue
        board = board_dir.relative_to(_BOARDS_DIR).as_posix()

        try:
            with cfg_path.open(encoding='utf-8') as f:
                cfg = json.load(f)

            manufacturer = _get_manufacturer(cfg)
            reported_type = _get_reported_type(cfg)
            target = cfg.get("target")
            if not isinstance(target, str) or not target:
                raise ValueError(f"{cfg_path}: missing non-empty target")

            previous_board = type_owners.get(reported_type)
            if previous_board is not None and previous_board != board:
                errors.append(
                    f"duplicate reported board type {reported_type!r} in "
                    f"{previous_board} and {board}"
                )
            else:
                type_owners[reported_type] = board

            if "/" in board:
                expected_manufacturer = board.split("/")[0]
                if not manufacturer:
                    errors.append(
                        f"{cfg_path}: Board is in '{expected_manufacturer}/' subdirectory, "
                        f"but config.json is missing \"manufacturer\": \"{expected_manufacturer}\""
                    )
                elif manufacturer != expected_manufacturer:
                    errors.append(
                        f"{cfg_path}: manufacturer mismatch, "
                        f"directory is '{expected_manufacturer}/' but config.json has \"{manufacturer}\""
                    )
            else:
                if manufacturer:
                    errors.append(
                        f"{cfg_path}: Board is not in a manufacturer subdirectory, "
                        f"but config.json defines manufacturer \"{manufacturer}\", "
                        f"please move board to main/boards/{manufacturer}/{board}/"
                    )

            builds = _get_builds_for_idf(cfg, idf_version)
            for build in builds:
                name = _get_reported_name(build)
                full_name = _get_release_full_name(manufacturer, build)

                previous_config = name_owners.get(name)
                if previous_config is not None:
                    errors.append(
                        f"duplicate reported board name {name!r} in "
                        f"{previous_config} and {cfg_path}"
                    )
                else:
                    name_owners[name] = str(cfg_path)

                identity = (reported_type, name)
                previous_config = identity_owners.get(identity)
                if previous_config is not None:
                    errors.append(
                        f"duplicate reported board identity {identity!r} in "
                        f"{previous_config} and {cfg_path}"
                    )
                else:
                    identity_owners[identity] = str(cfg_path)

                variants.append({
                    "board": board,
                    "name": name,
                    "full_name": full_name,
                    "type": reported_type,
                    "target": target,
                })

        except Exception as e:
            errors.append(f"{cfg_path}: {e}")

    seen_names: dict[str, str] = {}
    for variant in variants:
        previous_board = seen_names.get(variant["full_name"])
        if previous_board is not None:
            errors.append(
                f"duplicate artifact name {variant['full_name']!r} in "
                f"{previous_board} and {variant['board']}"
            )
        else:
            seen_names[variant["full_name"]] = variant["board"]

    if errors:
        details = "\n".join(f"  - {error}" for error in errors)
        raise ValueError(f"Invalid board configuration:\n{details}")

    for variant in variants:
        cfg_path = _BOARDS_DIR / variant["board"] / config_filename
        with cfg_path.open(encoding="utf-8") as file:
            cfg = json.load(file)
        build = next(
            item for item in _get_builds_for_idf(cfg, idf_version)
            if _get_reported_name(item) == variant["name"]
        )
        sdkconfig_append = build.get("sdkconfig_append", [])
        if not isinstance(sdkconfig_append, list) or not all(
            isinstance(item, str) for item in sdkconfig_append
        ):
            raise ValueError(
                f'{cfg_path}: build {variant["name"]!r} '
                'sdkconfig_append must be a string list'
            )
        config_symbol = _resolve_board_config(
            variant["board"],
            variant["target"],
            sdkconfig_append,
            variant_name=variant["name"],
        )
        variant["config"] = config_symbol
        variant["display_name"] = _get_board_display_name(config_symbol)
        variant["wake_word_supported"] = _board_supports_wake_word(
            variant["target"],
            sdkconfig_append,
        )
        variant["build_options"] = _build_option_definitions(
            variant["board"],
            variant["target"],
            config_symbol,
            build,
        )

    return sorted(variants, key=lambda variant: (variant["board"], variant["name"]))


_CI_REPRESENTATIVE_VARIANTS = Path("scripts/ci/representative-variants.json")


def _load_representative_variants(
    variants: list[dict[str, Any]],
    path: Path = _CI_REPRESENTATIVE_VARIANTS,
) -> list[dict[str, Any]]:
    """Resolve the reviewed CI list against the current SDK's valid variants."""
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict) or manifest.get("schema_version") != 1:
        raise ValueError(f"{path}: expected schema_version 1")
    entries = manifest.get("variants")
    if not isinstance(entries, list) or not entries:
        raise ValueError(f"{path}: variants must be a non-empty list")
    available = {(item["board"], item["name"]): item for item in variants}
    selected: list[dict[str, Any]] = []
    seen: set[tuple[str, str]] = set()
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError(f"{path}: each variant must be an object")
        unknown = set(entry) - {"board", "name", "reason", "build_options"}
        if unknown:
            raise ValueError(f"{path}: unknown fields: {', '.join(sorted(unknown))}")
        for field in ("board", "name", "reason"):
            if not isinstance(entry.get(field), str) or not entry[field].strip():
                raise ValueError(f"{path}: missing non-empty {field}")
        identity = (entry["board"], entry["name"])
        if identity in seen:
            raise ValueError(f"{path}: duplicate representative variant {identity}")
        seen.add(identity)
        if identity not in available:
            raise ValueError(
                f"{path}: representative variant {identity} is unavailable; "
                "check the board name and active ESP-IDF version (CI uses 6.1)"
            )
        variant = dict(available[identity])
        options = entry.get("build_options", {})
        _normalize_build_options(variant["build_options"], options)
        if options:
            variant["ci_build_options"] = options
            variant["ci_name"] = f"{variant['full_name']}-ci"
        selected.append(variant)
    return selected


def _select_variants_for_changes(
    variants: list[dict[str, Any]], changed_files: list[str]
) -> list[dict[str, Any]]:
    """Select representative builds plus every directly affected board variant."""
    known_boards = sorted({variant["board"] for variant in variants}, key=len, reverse=True)
    affected: set[str] = set()
    common_changed = False
    global_paths = {
        ".github/workflows/build.yml",
        "CMakeLists.txt",
        "scripts/build_default_assets.py",
        "scripts/build.py",
        "scripts/gen_lang.py",
        "scripts/versions.py",
        "dependencies.lock",
    }

    for raw_path in changed_files:
        path = raw_path.strip()
        if not path:
            continue
        if (path in global_paths or path.startswith("components/") or
                path.startswith("partitions/") or
                path.startswith("sdkconfig.defaults") or
                path.startswith("scripts/ci/") or
                (path.startswith("main/") and not path.startswith("main/boards/")) or
                path.startswith("main/boards/common/")):
            common_changed = True
            continue

        prefix = "main/boards/"
        if path.startswith(prefix):
            relative = path[len(prefix):]
            board = next(
                (candidate for candidate in known_boards
                 if relative == candidate or relative.startswith(f"{candidate}/")),
                None,
            )
            if board is not None:
                affected.add(board)

    selected = _load_representative_variants(variants) if common_changed else []
    selected_defaults = {
        (variant["board"], variant["name"])
        for variant in selected if not variant.get("ci_build_options")
    }
    selected.extend(
        variant for variant in variants
        if variant["board"] in affected
        and (variant["board"], variant["name"]) not in selected_defaults
    )
    return selected


def _find_board_config_candidates(board_type: str) -> list[str]:
    """Find all CONFIG_BOARD_TYPE_xxx candidates for the given board_type."""
    board_path = board_type.strip("/")
    lines = Path("main/CMakeLists.txt").read_text(encoding="utf-8").splitlines()
    candidates: list[str] = []
    branch_symbol: Optional[str] = None
    branch_lines: list[str] = []

    def finish_branch() -> None:
        if branch_symbol is None:
            return
        branch = "\n".join(branch_lines)
        directory_match = re.search(r'set\(BOARD_DIR\s+"([^"]+)"\)', branch)
        directory = directory_match.group(1) if directory_match else None
        if directory == board_path:
            candidates.append(branch_symbol)

    condition_pattern = re.compile(r"^\s*(?:if|elseif)\(([^)]+)\)")
    for line in lines:
        condition_match = condition_pattern.match(line)
        if condition_match:
            finish_branch()
            condition = condition_match.group(1)
            branch_symbol = (
                condition if condition.startswith("CONFIG_BOARD_TYPE_") else None
            )
            branch_lines = []
        elif re.match(r"^\s*(?:else|endif)\b", line):
            finish_branch()
            branch_symbol = None
            branch_lines = []
        elif branch_symbol is not None:
            branch_lines.append(line)
    finish_branch()

    return candidates


def _extract_board_config_from_sdkconfig_append(sdkconfig_append: list[str]) -> Optional[str]:
    """Extract explicit CONFIG_BOARD_TYPE_xxx=y from sdkconfig_append, if present."""
    pattern = re.compile(r"^(CONFIG_BOARD_TYPE_[A-Za-z0-9_]+)=y$")
    matches = []
    for item in sdkconfig_append:
        m = pattern.match(item.strip())
        if m:
            matches.append(m.group(1))
    if not matches:
        return None
    uniq = list(dict.fromkeys(matches))
    if len(uniq) > 1:
        raise ValueError(f"Multiple board type configs found in sdkconfig_append: {uniq}")
    return uniq[0]


def _board_config_symbol_exists(
    symbol: str,
    kconfig_path: Path = Path("main/Kconfig.projbuild"),
) -> bool:
    """Return whether a CONFIG_BOARD_TYPE_* symbol exists in project Kconfig."""
    if not kconfig_path.exists():
        return False
    name = symbol.removeprefix("CONFIG_")
    return bool(re.search(
        rf"^\s*config\s+{re.escape(name)}\s*$",
        kconfig_path.read_text(encoding="utf-8"),
        re.MULTILINE,
    ))


def _symbol_supports_target(symbol: str, target: str) -> bool:
    """Check whether Kconfig symbol depends on given target (e.g. esp32c5)."""
    kconfig_file = Path("main/Kconfig.projbuild")
    if not kconfig_file.exists():
        return False

    target_flag = f"IDF_TARGET_{target.upper()}"
    symbol = symbol.removeprefix("CONFIG_")
    lines = kconfig_file.read_text(encoding="utf-8").splitlines()

    in_symbol = False
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("config "):
            curr_symbol = stripped.split("config ", 1)[1].strip()
            in_symbol = curr_symbol == symbol
            continue
        if in_symbol and stripped.startswith(("config ", "choice ", "endchoice", "menu ", "endmenu")):
            break
        if (
            in_symbol
            and "depends on" in stripped
            and re.search(
                rf"(?<![A-Za-z0-9_]){re.escape(target_flag)}(?![A-Za-z0-9_])",
                stripped,
            )
        ):
            return True
    return False


def _resolve_board_config(
    board_type: str,
    target: str,
    sdkconfig_append: list[str],
    *,
    variant_name: Optional[str] = None,
) -> str:
    """Resolve CONFIG_BOARD_TYPE_xxx for current board build."""
    def validate_target(symbol: str) -> str:
        if not _symbol_supports_target(symbol, target):
            raise ValueError(
                f"Board config {symbol} for {board_type!r} does not support "
                f"target {target!r}"
            )
        return symbol

    explicit = _extract_board_config_from_sdkconfig_append(sdkconfig_append)
    candidates = _find_board_config_candidates(board_type)
    if not candidates:
        raise ValueError(f"Cannot find board config symbol for {board_type}")

    if explicit and _board_config_symbol_exists(explicit):
        if explicit not in candidates:
            raise ValueError(
                f"Board config {explicit} does not select board directory "
                f"{board_type!r}"
            )
        return validate_target(explicit)
    if explicit:
        print(
            f"[WARN] Explicit board config {explicit} does not exist in Kconfig; "
            f"resolving it from BOARD_DIR={board_type!r} instead.",
            file=sys.stderr,
        )

    if len(candidates) == 1:
        return validate_target(candidates[0])

    if variant_name:
        expected = "CONFIG_BOARD_TYPE_" + re.sub(
            r"[^A-Z0-9]+",
            "_",
            variant_name.upper(),
        ).strip("_")
        by_variant = [candidate for candidate in candidates if candidate == expected]
        if len(by_variant) == 1:
            return validate_target(by_variant[0])

    by_target = [c for c in candidates if _symbol_supports_target(c, target)]
    if len(by_target) == 1:
        return validate_target(by_target[0])
    if len(by_target) > 1:
        selected = by_target[0]
        print(
            f"[WARN] Ambiguous board config for {board_type} (target={target}), "
            f"target-matched candidates={by_target}, selecting first: {selected}",
            file=sys.stderr,
        )
        return validate_target(selected)

    raise ValueError(
        f"Cannot resolve board config for {board_type}: "
        f"candidates={candidates}, target={target}"
    )


def _merge_sdkconfig_options(
    base: list[str],
    override: list[str],
) -> list[str]:
    """Merge two sdkconfig fragments, with override taking precedence."""
    result = list(base)
    for item in override:
        key = item.strip().split("=", 1)[0]
        result = [line for line in result if not line.startswith(f"{key}=")]
        result.append(item)
    return result


def main():
    parser = argparse.ArgumentParser(description="Xiaozhi Assistant build script")
    parser.add_argument("board", help="Board type to build")
    parser.add_argument("--name", default=None, help="Board name/variant")
    parser.add_argument("--language", default=_DEFAULT_LANGUAGE, help="Language")
    parser.add_argument("--wake-word", default=None, help="Wake word model")
    parser.add_argument("--preview", action="store_true", help="Preview only")
    args = parser.parse_args()

    # This is a simplified version - the full script has many more functions
    print(f"Building {args.board}...")
    print(f"  language: {args.language}")
    if args.wake_word:
        print(f"  wake_word: {args.wake_word}")

    _emit_build_stage("configure")
    _run_idf("reconfigure", preview=args.preview)
    _emit_build_stage("build")
    _run_idf("build", preview=args.preview)


if __name__ == "__main__":
    main()
