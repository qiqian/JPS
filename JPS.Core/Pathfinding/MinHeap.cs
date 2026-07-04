/*
 * MinHeap.cs
 * JPS Pathfinding
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
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
            while (i > 0)
            {
                int parent = (i - 1) >> 1;
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
            while (true)
            {
                int l = (i << 1) + 1;
                int r = l + 1;
                int child;

                if (l >= _count)
                    break;

                child = l;
                if (r < _count && _prio[r] < _prio[l])
                    child = r;
                if (_prio[child] >= priority)
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
