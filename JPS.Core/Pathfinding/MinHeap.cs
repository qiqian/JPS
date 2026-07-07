/*
 * MinHeap.cs
 * JPS Pathfinding
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

using System;

namespace JPS.Pathfinding
{
    /// <summary>
    /// 四叉（4-ary）最小堆：元素为 int（节点 id），优先级为 long（f 值）。
    /// 用来替代 .NET 6+ 的 <c>PriorityQueue&lt;TElement,TPriority&gt;</c>，以兼容 Unity 2022 / netstandard2.1。
    /// 四叉树高 ≈ log4(n)（比二叉减半），四个孩子在数组内连续、cache 局部性更好。
    /// 与 C 版 min_heap.c 严格一致（同 d=4、同 sift 逻辑），保证 C≡C# 出队顺序逐位相同。
    /// </summary>
    public sealed class MinHeap
    {
        private int[] _elem;
        private long[] _prio;
        private int _count;

        public MinHeap(int capacity = 64)
        {
            if (capacity < 1) capacity = 1;
            _elem = new int[capacity];
            _prio = new long[capacity];
        }

        public int Count => _count;

        public void Clear() => _count = 0;

        public void Enqueue(int element, long priority)
        {
            if (_count == _elem.Length)
                Grow();

            int i = _count++;
            // hole sift-up（4-ary）：parent(i) = (i-1)/4，层数比二叉减半。
            while (i > 0)
            {
                int parent = (i - 1) >> 2;
                if (_prio[parent] <= priority)
                    break;

                _elem[i] = _elem[parent];
                _prio[i] = _prio[parent];
                i = parent;
            }

            _elem[i] = element;
            _prio[i] = priority;
        }

        public bool TryDequeue(out int element, out long priority)
        {
            if (_count == 0)
            {
                element = 0;
                priority = 0;
                return false;
            }

            element = _elem[0];
            priority = _prio[0];
            _count--;

            if (_count > 0)
                SiftDown(0, _elem[_count], _prio[_count]);

            return true;
        }

        private void SiftDown(int i, int element, long priority)
        {
            // 四叉 sift-down：四个孩子 4i+1..4i+4 在数组内连续，每层顺序扫出最小孩子。
            // 层数减半，代价是每层最多 3 次孩子间比较。挑选顺序与 C 版 min_heap.c 逐位一致。
            while (true)
            {
                int baseChild = (i << 2) + 1;   // 第一个孩子 = 4i+1
                if (baseChild >= _count)
                    break;

                int child = baseChild;
                long best = _prio[baseChild];
                int limit = baseChild + 4;
                if (limit > _count)
                    limit = _count;
                for (int c = baseChild + 1; c < limit; c++)
                {
                    if (_prio[c] < best)
                    {
                        best = _prio[c];
                        child = c;
                    }
                }
                if (best >= priority)
                    break;

                _elem[i] = _elem[child];
                _prio[i] = _prio[child];
                i = child;
            }

            _elem[i] = element;
            _prio[i] = priority;
        }

        private void Grow()
        {
            int n = _elem.Length * 2;
            Array.Resize(ref _elem, n);
            Array.Resize(ref _prio, n);
        }
    }
}
