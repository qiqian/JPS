using System;
using System.Globalization;

namespace JPS.Controls;

/// <summary>
/// 极简界面本地化：按当前系统/界面语言在中、英之间二选一。
/// 语言为 zh*（zh / zh-CN / zh-TW …）时用中文，其它一律英文。
///
/// 仅服务于 UI 层（工具栏按钮、图例、状态栏、对话框），刻意不进入
/// Pathfinding / Models 等可移植算法层，保持算法核心与界面/语言无关。
/// </summary>
public static class Loc
{
    /// <summary>当前界面语言是否中文。</summary>
    public static readonly bool Zh =
        CultureInfo.CurrentUICulture.TwoLetterISOLanguageName
            .Equals("zh", StringComparison.OrdinalIgnoreCase);

    /// <summary>按语言二选一返回字符串。</summary>
    public static string T(string zh, string en) => Zh ? zh : en;
}
