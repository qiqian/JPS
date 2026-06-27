using System;
using System.IO;
using JPS.Models;

namespace JPS.Data
{
    /// <summary>
    /// MovingAI 基准地图（.map）解析器。
    ///
    /// 文件格式（octile 网格）：
    /// <code>
    /// type octile
    /// height H
    /// width  W
    /// map
    /// （随后 H 行、每行 W 个字符的地形）
    /// </code>
    /// 地形字符按 MovingAI 约定二值化为“可走 / 阻挡”：
    ///   可走 = '.'（普通）、'G'（普通）、'S'（沼泽，可从陆地进入）；其余（'@' 'O' 出界、'T' 树、'W' 水…）均视为阻挡。
    /// </summary>
    public static class MovingAiMap
    {
        public static bool IsWalkableTerrain(char c) => c == '.' || c == 'G' || c == 'S';

        /// <summary>从 .map 文本解析为 <see cref="GridMap"/>（阻挡按地形二值化）。</summary>
        public static GridMap Parse(string text)
        {
            if (text == null) throw new ArgumentNullException(nameof(text));

            using var reader = new StringReader(text);
            int width = 0, height = 0;
            bool sawMap = false;

            string? line;
            while ((line = reader.ReadLine()) != null)
            {
                string t = line.Trim();
                if (t.Length == 0) continue;

                if (t.Equals("map", StringComparison.OrdinalIgnoreCase)) { sawMap = true; break; }
                else if (StartsWithKeyword(t, "height")) height = ParseValue(t, "height");
                else if (StartsWithKeyword(t, "width")) width = ParseValue(t, "width");
                // "type octile" 及其它未知头部行直接忽略
            }

            if (!sawMap || width <= 0 || height <= 0)
                throw new FormatException("不是有效的 MovingAI .map 文件（缺少 width/height/map 头）。");

            var map = new GridMap(width, height);
            for (int y = 0; y < height; y++)
            {
                line = reader.ReadLine();
                if (line == null) break;   // 文件被截断：剩余行默认全可走

                int n = Math.Min(width, line.Length);
                for (int x = 0; x < n; x++)
                    if (!IsWalkableTerrain(line[x]))
                        map.SetBlocked(x, y, true);
                // 行长不足 width 的尾部格子默认可走（GridMap 初值即可走）
            }

            return map;
        }

        private static bool StartsWithKeyword(string line, string keyword) =>
            line.Length > keyword.Length &&
            line.StartsWith(keyword, StringComparison.OrdinalIgnoreCase) &&
            (line[keyword.Length] == ' ' || line[keyword.Length] == '\t');

        private static int ParseValue(string line, string keyword)
        {
            string rest = line.Substring(keyword.Length).Trim();
            return int.Parse(rest);
        }
    }
}
