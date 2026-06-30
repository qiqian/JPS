import os

def batch_add_utf8_bom(directory):
    # 你可以根据需要增删文件后缀
    target_extensions = ('.cpp', '.h', '.c', '.hpp', '.cc')
    converted_count = 0

    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(target_extensions):
                filepath = os.path.join(root, file)
                
                # 1. 尝试读取文件（先试 UTF-8，报错说明可能是 GBK）
                try:
                    with open(filepath, 'r', encoding='utf-8') as f:
                        content = f.read()
                except UnicodeDecodeError:
                    # 如果 UTF-8 读取失败，降级使用 GBK (代码页 936) 读取
                    with open(filepath, 'r', encoding='gbk') as f:
                        content = f.read()
                
                # 2. 以带 BOM 的 UTF-8 (utf-8-sig) 格式覆写回去
                with open(filepath, 'w', encoding='utf-8-sig') as f:
                    f.write(content)
                
                print(f"[成功] 已转换: {filepath}")
                converted_count += 1
                
    print(f"\n处理完毕！共统一了 {converted_count} 个文件的编码。")

if __name__ == '__main__':
    # 默认处理运行该脚本的当前目录
    current_dir = os.getcwd()
    print(f"正在扫描目录: {current_dir} ...\n")
    batch_add_utf8_bom(current_dir)