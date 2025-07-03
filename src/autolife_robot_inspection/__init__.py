import os
import toml
import pathlib

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir))
PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__)))

if os.path.exists(os.path.abspath(os.path.join(PROJECT_ROOT, 'configs'))):
    CONFIGS_ROOT = os.path.abspath(os.path.join(PROJECT_ROOT, 'configs'))
elif os.path.exists(os.path.abspath(os.path.join(PACKAGE_ROOT, 'configs'))):
    CONFIGS_ROOT = os.path.abspath(os.path.join(PACKAGE_ROOT, 'configs'))
else:
    raise FileNotFoundError("Configs directory not found in PROJECT_ROOT or PACKAGE_ROOT.")

if os.path.exists(os.path.abspath(os.path.join(PROJECT_ROOT, 'models'))):
    MODELS_ROOT = os.path.abspath(os.path.join(PROJECT_ROOT, 'models'))
elif os.path.exists(os.path.abspath(os.path.join(PACKAGE_ROOT, 'models'))):
    MODELS_ROOT = os.path.abspath(os.path.join(PACKAGE_ROOT, 'models'))
else:
    raise FileNotFoundError("Models directory not found in PROJECT_ROOT or PACKAGE_ROOT.")

ONNX_ROOT = os.path.join(MODELS_ROOT, 'MediumOnnx')
PKL_ROOT = os.path.join(MODELS_ROOT, 'pkl')

PROGRAM_SETTINGS = None
with open(os.path.join(PACKAGE_ROOT, "settings.toml"), "r") as f:
    PROGRAM_SETTINGS = toml.load(f)

try:
    from autolife_robot_sdk import GLOBAL_VARS, reload_sdk_constants
    GLOBAL_VARS.ACTIVE_ROBOT_VERSION = PROGRAM_SETTINGS["sdk_settings"]["active_robot_version"]
    reload_sdk_constants()
except ImportError:
    raise ImportError("Please install the autolife_robot_sdk package to use this module.")

MENU_CONFIG_PATH = None
FUNC_CONFIG_PATH = None
MODEL_CONFIG_PATH = None
UI_CSS_PATH = None

def load_vision():
    global MENU_CONFIG_PATH, FUNC_CONFIG_PATH, MODEL_CONFIG_PATH, UI_CSS_PATH
    MODEL_CONFIG_PATH = pathlib.Path(f'{CONFIGS_ROOT}/model_config.json')
    MENU_CONFIG_PATH = pathlib.Path(f'{CONFIGS_ROOT}/menu_config.json')
    FUNC_CONFIG_PATH = pathlib.Path(f'{CONFIGS_ROOT}/func_config.json')
    UI_CSS_PATH = pathlib.Path(f'{CONFIGS_ROOT}/inspection_ui.tcss')

# Call the function to load the appropriate value based on the ACTIVE_ROBOT_VERSION
load_vision()
