import getopt
import os
import re
import sys
import platform
import traceback
from distutils.core import setup

from Cython.Build import cythonize

filtered_file = [r".*?__init__\.py", r".*?test.*?", r".*?main\.py",
                 r".*?cythonizer.py", r".*?setup.py"]
exclude_pattern = [re.compile(f) for f in filtered_file]


def __print_progress_bar(iteration, total, prefix='', suffix='', decimals=1, length=100, fill='█', printEnd="\r\n"):
    """
    Call in a loop to create terminal progress bar

    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        length      - Optional  : character length of bar (Int)
        fill        - Optional  : bar fill character (Str)
        printEnd    - Optional  : end character (e.g. "\r", "\r\n") (Str)
    """
    try:
        percent = ("{0:." + str(decimals) + "f}").format(100 *
                                                         (iteration / float(total)))
        filledLength = int(length * iteration // total)
        bar = fill * filledLength + '-' * (length - filledLength)
        print(f'\r{prefix} |{bar}| {percent}% {suffix}', end=printEnd)
        # Print New Line on Complete
        if iteration == total:
            print()
    except Exception as e:
        pass


def __check_if_file_is_excluded(full_file_path):
    for pattern in exclude_pattern:
        if pattern.match(full_file_path):
            return True
    return False


def __get_compiled_file_list(src_dir):
    compiled_file_list = []
    for root, dirs, files in os.walk(src_dir, topdown=True):
        for filename in files:
            if filename.endswith('.py') and filename not in filtered_file:
                full_file_path = os.path.join(root, filename)
                if not __check_if_file_is_excluded(full_file_path):
                    compiled_file_list.append(full_file_path)
    return compiled_file_list


def __compile(file_list, debug=False):
    print("*****************************")
    print("****** Start Compiling ******")
    print("*****************************")
    if debug:
        __print_progress_bar(
            0, len(file_list), prefix='Progress:', suffix='Complete', length=50)
        for i, f in enumerate(file_list):
            setup(ext_modules=cythonize(f, compiler_directives={"language_level": "3"}),
                  script_args=['build_ext'],
                  options={
                'build_ext': {
                    'inplace': True,
                }
            })
            __print_progress_bar(
                i + 1, len(file_list), prefix='Progress:', suffix='Complete', length=50)
    else:
        setup(ext_modules=cythonize(file_list, nthreads=32, compiler_directives={"language_level": "3"}),
              script_args=['build_ext'],
              options={
            'build_ext': {
                'inplace': True,
            }
        })


def __clear_py_file(file_list):
    print("*************************************")
    print("****** Start Cleaning .py File ******")
    print("*************************************")
    __print_progress_bar(0, len(file_list), prefix='Progress:',
                         suffix='Complete', length=50)
    for i, file in enumerate(file_list):
        if os.path.exists(file):
            os.remove(file)
            print(f"{file} is removed")
        __print_progress_bar(i + 1, len(file_list),
                             prefix='Progress:', suffix='Complete', length=50)


def __clear_c_file(file_list):
    print("************************************")
    print("****** Start Cleaning .c File ******")
    print("************************************")
    __print_progress_bar(0, len(file_list), prefix='Progress:',
                         suffix='Complete', length=50)
    for i, file in enumerate(file_list):
        if os.path.exists(os.path.splitext(file)[0] + '.c'):
            os.remove(os.path.splitext(file)[0] + '.c')
            print(f"{os.path.splitext(file)[0] + '.c'} is removed")
        __print_progress_bar(i + 1, len(file_list),
                             prefix='Progress:', suffix='Complete', length=50)


def __clear_binary_file(file_list):
    print("**************************************")
    print("****** Start Cleaning .pyd/.so File ******")
    print("**************************************")
    __print_progress_bar(0, len(file_list), prefix='Progress:',
                         suffix='Complete', length=50)
    file_ext = ''
    if os.name == 'nt':
        file_ext = f".cp{sys.version_info.major}{sys.version_info.minor}-win_amd64.pyd"
    elif os.name == 'posix':
        if platform.uname().machine.lower() == 'amd64':
            file_ext = f".cpython-{sys.version_info.major}{sys.version_info.minor}-amd64-linux-gnu.so"
        elif platform.uname().machine.lower() == 'aarch64':
            file_ext = f".cpython-{sys.version_info.major}{sys.version_info.minor}-aarch64-linux-gnu.so"
    else:
        raise Exception('OS not supported')

    for i, file in enumerate(file_list):
        if os.path.exists(os.path.splitext(file)[0] + file_ext):
            os.remove(os.path.splitext(file)[0] + file_ext)
            print(
                f"{os.path.splitext(file)[0] + file_ext} is removed")
        __print_progress_bar(i + 1, len(file_list),
                             prefix='Progress:', suffix='Complete', length=50)


def __print_help():
    print("To debug:")
    print("python _cythonizer.py -d")
    print("To clean .pyd/.so and .c files:")
    print("python _cythonizer.py -c")
    print("To specify source directory:")
    print("python _cythonizer.py -s <source_directory>")


def main(argv):
    debug = False
    cleaning = False
    src_dir = os.path.abspath(os.path.join(__file__, os.pardir))  # Default src_dir

    try:
        opts, args = getopt.getopt(argv, "hdcs:", ["help", "debug", "clean", "src="])
    except getopt.GetoptError as e:
        print(f"Error: {e}")
        __print_help()
        return 160

    for opt, arg in opts:
        if opt in ("-h", "--help"):
            __print_help()
            sys.exit(0)
        elif opt in ("-d", "--debug"):
            debug = True
        elif opt in ("-c", "--clean"):
            cleaning = True
        elif opt in ("-s", "--src"):
            src_dir = os.path.abspath(arg)
            if not os.path.isdir(src_dir):
                print(f"Error: Source directory '{src_dir}' not found or is not a directory.")
                return 1

    print(f"Using source directory: {src_dir}")

    try:
        # src_dir = os.path.abspath(os.path.join(__file__, os.pardir)) # Moved default assignment up
        file_list = __get_compiled_file_list(src_dir)
        if not file_list:
            print(f"No Python files found to process in {src_dir}")
            return 0

        if cleaning:
            __clear_c_file(file_list)
            __clear_binary_file(file_list)
        else:
            try:
                __compile(file_list, debug)
            except Exception as e:
                print("Compilation failed:")
                print(traceback.format_exc())
                __print_help()
                return 1

            if not debug:
                __clear_c_file(file_list)
                __clear_py_file(file_list)
    except Exception as e:
        print("An unexpected error occurred:")
        print(traceback.format_exc())
        __print_help()
        return 1
    return 0


if __name__ == "__main__":
    error_code = main(sys.argv[1:])
    sys.exit(error_code)
