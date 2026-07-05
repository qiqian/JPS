import os
import sys

# 递归把源码文件统一成「带 BOM 的 UTF-8」。os.walk 本就递归子目录；这里额外：
#   ① 可传目标目录参数（默认脚本所在目录），方便指向任意子树；
#   ② 排除 JPS.Native/ndk（下载的 NDK，成千上万第三方头）与 JPS.Native/build-android*（Android 构建产物）
#      ——否则递归会把这些第三方/生成文件也一并改写（既没意义又可能弄坏 NDK）。

TARGET_EXTENSIONS = ('.cpp', '.h', '.c', '.hpp', '.cc')


def should_skip_dir(name):
    # 只排除下载的 NDK 与 Android 构建产物目录。
    return name == 'ndk' or name.startswith('build-android')


def batch_add_utf8_bom(directory):
    converted_count = 0
    skipped_dir_count = 0

    for root, dirs, files in os.walk(directory):
        # 就地裁剪 dirs，让 os.walk 不再递归进被跳过的子目录（比之后再判断更省）。
        keep = [d for d in dirs if not should_skip_dir(d)]
        skipped_dir_count += len(dirs) - len(keep)
        dirs[:] = keep

        for file in files:
            if not file.endswith(TARGET_EXTENSIONS):
                continue
            filepath = os.path.join(root, file)

            # 1. 读取：先试 UTF-8-SIG（会吞掉已有 BOM，保证重复运行不叠 BOM），失败再退回 GBK(cp936)。
            try:
                with open(filepath, 'r', encoding='utf-8-sig') as f:
                    content = f.read()
            except UnicodeDecodeError:
                try:
                    with open(filepath, 'r', encoding='gbk') as f:
                        content = f.read()
                except UnicodeDecodeError:
                    print(f"[跳过] 无法按 UTF-8/GBK 解码: {filepath}")
                    continue

            # 2. 以带 BOM 的 UTF-8 (utf-8-sig) 覆写回去。
            with open(filepath, 'w', encoding='utf-8-sig') as f:
                f.write(content)

            print(f"[成功] 已转换: {filepath}")
            converted_count += 1

    print(f"\n处理完毕！共统一了 {converted_count} 个文件的编码，跳过 {skipped_dir_count} 个构建/第三方目录。")


if __name__ == '__main__':
    # 目标目录：命令行第一个参数，否则默认脚本所在目录（不随 cwd 变化，双击/别处运行都稳定）。
    if len(sys.argv) > 1:
        target_dir = os.path.abspath(sys.argv[1])
    else:
        target_dir = os.path.dirname(os.path.abspath(__file__))

    if not os.path.isdir(target_dir):
        print(f"错误：不是目录：{target_dir}")
        sys.exit(1)

    print(f"正在递归扫描目录: {target_dir} ...\n")
    batch_add_utf8_bom(target_dir)
