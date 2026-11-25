#!/usr/bin/env python3
"""上传后的完整性自检：换行符 / BOM / 语法 / 关键文件 / 中文文件名。

在 Orin 上第一次进入工程目录时跑一次，比等 launch 报错再回头找省事：

    python3 scripts/verify_tree.py

退出码 0 = 可以继续下一步；1 = 有阻塞问题，按提示修。

为什么用 Python 而不是 shell 来写这个检查：脚本自己也可能被 CRLF 污染
（bash 会在第二行就崩），而 Python 能容忍 CRLF，所以这个检查在任何情况下都跑得起来。
"""
import ast
import os
import shutil
import subprocess
import sys
import unicodedata

TEXT_EXT = ('.sh', '.py', '.yaml', '.yml', '.cpp', '.hpp', '.h', '.msg',
            '.txt', '.xml', '.md', '.service', '.rules', '.cfg', '.in')
SKIP_DIRS = {'build', 'install', 'log', '.git', '__pycache__', 'firmware', 'operator_ui'}

BASH_SCRIPTS = ['scripts/install_deps.sh', 'scripts/preflight.sh', 'scripts/check.sh']
REQUIRED = [
    'README.md',
    'src/uw_robot_bringup/package.xml',
    'src/uw_robot_bringup/CMakeLists.txt',
    'src/uw_robot_bringup/config/robot.yaml',
    'src/uw_robot_bringup/launch/robot.launch.py',
    'src/uw_robot_bringup/launch/depth.launch.py',
    'src/uw_robot_communication/package.xml',
    'src/uw_robot_control/package.xml',
    'src/uw_robot_control/test/closed_loop/mode_switch_test.cpp',
    'src/uw_robot_interfaces/package.xml',
    'src/uw_robot_sensors/package.xml',
    'src/uw_robot_sensors/setup.py',
    'src/uw_robot_sensors/setup.cfg',
    'src/uw_robot_sensors/resource/uw_robot_sensors',
    'src/uw_robot_sensors/uw_robot_sensors/depth/__init__.py',
    'src/uw_robot_sensors/uw_robot_sensors/depth/ms5837.py',
    'src/uw_robot_sensors/uw_robot_sensors/depth/driver.py',
    'src/uw_robot_sensors/uw_robot_sensors/depth/depth_node.py',
    'src/uw_robot_sensors/test/test_depth.py',
]
# 中文文件名最容易在 Windows→Linux 解压时变成乱码，逐个点名核对。
CJK_FILES = [
    'docs/Orin部署.md',
    'docs/MS5837直连Orin.md',
    'docs/OpenCV双目相机.md',
    'docs/实物迁移与标定.md',
    'docs/直接运行.md',
    'docs/结构和接口.md',
    'docs/工程规范.md',
]

problems = []
notes = []
crlf_files = set()


def ok(msg):
    print('  \033[32m✓\033[0m %s' % msg)


def warn(msg):
    print('  \033[33m!\033[0m %s' % msg)


def bad(msg):
    print('  \033[31m✗\033[0m %s' % msg)
    problems.append(msg)


def walk():
    for root, dirs, files in os.walk('.'):
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        for name in files:
            yield os.path.join(root, name)


def check_eol():
    print('== 1. 换行符与 BOM ==')
    crlf, bom = [], []
    for path in walk():
        if not path.endswith(TEXT_EXT):
            continue
        try:
            raw = open(path, 'rb').read()
        except OSError:
            continue
        if raw.startswith(b'\xef\xbb\xbf'):
            bom.append(path)
        if b'\r\n' in raw:
            crlf.append(path)
    if crlf:
        crlf_files.update(crlf)
        bad('%d 个文件是 CRLF 换行：%s' % (len(crlf), ', '.join(sorted(crlf)[:5])))
        print('      修复：find . -path ./build -prune -o -path ./install -prune -o '
              '-type f \\( -name "*.sh" -o -name "*.py" -o -name "*.yaml" \\) '
              "-exec sed -i 's/\\r$//' {} +")
    else:
        ok('全部 %d 个文本文件都是 LF' % sum(1 for p in walk() if p.endswith(TEXT_EXT)))
    if bom:
        bad('%d 个文件带 UTF-8 BOM：%s' % (len(bom), ', '.join(sorted(bom))))
        print("      修复：sed -i '1s/^\\xef\\xbb\\xbf//' <文件>")
    else:
        ok('无 UTF-8 BOM（BOM 会让 shebang 失效、YAML 首键解析失败）')


def check_required():
    print('== 2. 关键文件 ==')
    missing = [p for p in REQUIRED if not os.path.isfile(p)]
    if missing:
        bad('缺少 %d 个文件：%s' % (len(missing), ', '.join(missing)))
    else:
        ok('%d 个必需文件齐全（5 个包的 package.xml、深度节点三件套、测试）' % len(REQUIRED))
    lost = [p for p in CJK_FILES if not os.path.isfile(p)]
    if lost:
        bad('中文文件名丢失/乱码：%s —— 解压工具的编码问题，'
            '改用 tar 或 7-Zip 重传' % ', '.join(lost))
    else:
        ok('%d 个中文名文档正常' % len(CJK_FILES))


def check_bash():
    print('== 3. Shell 脚本语法 ==')
    bash = shutil.which('bash')
    if not bash:
        warn('没有 bash，跳过')
        return
    flagged = {os.path.normpath(p).replace('\\', '/') for p in crlf_files}
    for path in BASH_SCRIPTS:
        if not os.path.isfile(path):
            continue
        if os.path.normpath(path).replace('\\', '/') in flagged:
            warn('%s 含 CRLF，先按第 1 节修换行再验（bash -n 查不出 CRLF）' % path)
            continue
        proc = subprocess.run([bash, '-n', path], capture_output=True, text=True)
        if proc.returncode == 0:
            ok('%s 语法通过' % path)
        else:
            first = (proc.stderr.strip().splitlines() or ['(无输出)'])[0]
            bad('%s 语法错误：%s' % (path, first))
    for path in BASH_SCRIPTS:
        if os.path.isfile(path) and not os.access(path, os.X_OK) and os.name == 'posix':
            notes.append('%s 没有执行位（用 bash 调用不受影响）' % path)


def check_python():
    print('== 4. Python 语法 ==')
    total, failed = 0, []
    for path in walk():
        if not path.endswith('.py'):
            continue
        total += 1
        try:
            ast.parse(open(path, encoding='utf-8').read(), path)
        except SyntaxError as exc:
            failed.append('%s:%s %s' % (path, exc.lineno, exc.msg))
    if failed:
        bad('%d 个 Python 文件语法错误：%s' % (len(failed), '; '.join(failed)))
    else:
        ok('%d 个 Python 文件语法通过' % total)


def check_yaml():
    print('== 5. YAML 与消息定义 ==')
    yamls = [p for p in walk() if p.endswith(('.yaml', '.yml'))]
    try:
        import yaml
    except ImportError:
        warn('未安装 pyyaml，跳过解析（sudo apt install python3-yaml）')
    else:
        broken = []
        for path in yamls:
            try:
                yaml.safe_load(open(path, encoding='utf-8').read())
            except Exception as exc:  # noqa: BLE001 - 只是报给人看
                broken.append('%s: %s' % (path, exc))
        if broken:
            bad('YAML 解析失败：%s' % '; '.join(broken))
        else:
            ok('%d 个 YAML 可解析' % len(yamls))

    msgs = [p for p in walk() if p.endswith('.msg')]
    blank = [p for p in msgs if not open(p, encoding='utf-8').read().strip()]
    if blank:
        bad('空的 .msg 文件（rosidl 会拒绝）：%s' % ', '.join(blank))
    else:
        ok('%d 个 .msg 非空' % len(msgs))


def yaml_section_keys(path, section):
    """取 `<section>: ros__parameters:` 下缩进最浅的一级键，不依赖 pyyaml。"""
    keys, inside, base = [], False, None
    for raw in open(path, encoding='utf-8'):
        line = raw.rstrip('\n')
        if not inside:
            if line.strip() == section + ':':
                inside = True
            continue
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        indent = len(line) - len(line.lstrip())
        if indent == 0:            # 跨到下一个顶层段，结束
            break
        if line.strip() == 'ros__parameters:':
            continue
        if base is None:
            base = indent
        if indent == base and ':' in line:
            keys.append(line.strip().split(':', 1)[0].strip())
    return keys


def check_depth_config():
    """robot.yaml 的 depth_node 段与 depth_node.py 声明的参数是否对得上。"""
    print('== 6. 深度节点参数一致性 ==')
    yaml_path = 'src/uw_robot_bringup/config/robot.yaml'
    node_path = 'src/uw_robot_sensors/uw_robot_sensors/depth/depth_node.py'
    try:
        params = set(yaml_section_keys(yaml_path, 'depth_node'))
    except OSError as exc:
        warn('读 %s 失败，跳过：%s' % (yaml_path, exc))
        return
    if not params:
        bad('robot.yaml 里没解析出 depth_node 参数（缩进或段名被改坏了）')
        return
    src = open(node_path, encoding='utf-8').read()
    declared = set()
    for line in src.splitlines():
        if 'declare_parameter("' in line:
            declared.add(line.split('declare_parameter("')[1].split('"')[0])
        elif 'declare("' in line:
            declared.add(line.split('declare("')[1].split('"')[0])
    unused = sorted(params - declared - {'use_sim_time'})
    undefined = sorted(declared - params)
    if undefined:
        notes.append('depth_node 里有默认值但 yaml 未给（不阻塞）：%s' % ', '.join(undefined))
    if unused:
        bad('robot.yaml 里 depth_node 的 %s 没有被节点声明（拼错或已废弃）'
            % ', '.join(unused))
    else:
        ok('yaml 中 %d 个 depth_node 参数全部被节点声明' % len(params))


def main():
    if not os.path.isfile('src/uw_robot_bringup/config/robot.yaml'):
        print('当前目录不是工程根目录（找不到 src/uw_robot_bringup/config/robot.yaml）')
        print('先 cd 到工程根目录再跑。')
        return 2
    print('工程根目录：%s\n' % os.path.abspath('.'))
    check_eol()
    check_required()
    check_bash()
    check_python()
    check_yaml()
    check_depth_config()

    print()
    for note in notes:
        warn(note)
    if problems:
        print('\033[31m%d 个阻塞问题\033[0m：修完重跑本脚本。' % len(problems))
        return 1
    print('\033[32m自检通过\033[0m：可以 bash scripts/install_deps.sh 了。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
