/*
 * MinHeap.cs
 * JPS Pathfinding
 * Copyright (c) 2026 Qian Qian. MIT License.
 */

using System;

namespace JPS.Pathfinding
{
    /// <summary>
    /// 二叉最小堆：元素为 int（节点 id），优先级为 long（f 值）。
    /// 用来替代 .NET 6+ 的 <c>PriorityQueue&lt;TElement,TPriority&gt;</c>，以兼容 Unity 2022 / netstandard2.1。
    /// 行为与性能与之等价（同为二叉堆，O(log n) 入队/出队）。
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
            _elem[i] = element;
            _prio[i] = priority;

            // 上浮
            while (i > 0)
            {
                int parent = (i - 1) >> 1;
                if (_prio[parent] <= _prio[i])
                    break;
                Swap(i, parent);
                i = parent;
            }
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
            {
                _elem[0] = _elem[_count];
                _prio[0] = _prio[_count];
                SiftDown(0);
            }

            return true;
        }

        private void SiftDown(int i)
        {
            while (true)
            {
                int l = (i << 1) + 1;
                int r = l + 1;
                int smallest = i;

                if (l < _count && _prio[l] < _prio[smallest]) smallest = l;
                if (r < _count && _prio[r] < _prio[smallest]) smallest = r;
                if (smallest == i)
                    break;

                Swap(i, smallest);
                i = smallest;
            }
        }

        private void Swap(int a, int b)
        {
            int e = _elem[a]; _elem[a] = _elem[b]; _elem[b] = e;
            long p = _prio[a]; _prio[a] = _prio[b]; _prio[b] = p;
        }

        private void Grow()
        {
            int n = _elem.Length * 2;
            Array.Resize(ref _elem, n);
            Array.Resize(ref _prio, n);
        }
    }
}
