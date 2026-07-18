using JPS.Models;
using JPS.Pathfinding;
using System.Runtime.InteropServices;
using Xunit;

namespace JPS.Core.Tests;

public sealed class JpsAdapterTests
{
    [Fact]
    public void StaticObstacleAndMapBoundaryAreExpanded()
    {
        var source = new GridMap(9, 9);
        source.SetBlocked(4, 4, true);

        var adapter = new JpsAdapter(source, obstaclePadding: 1);

        for (int y = 3; y <= 5; y++)
            for (int x = 3; x <= 5; x++)
                Assert.True(adapter.Map.IsBlocked(x, y));

        for (int i = 0; i < 9; i++)
        {
            Assert.True(adapter.Map.IsBlocked(i, 0));
            Assert.True(adapter.Map.IsBlocked(i, 8));
            Assert.True(adapter.Map.IsBlocked(0, i));
            Assert.True(adapter.Map.IsBlocked(8, i));
        }

        Assert.True(adapter.Map.IsWalkable(2, 2));
        Assert.True(adapter.IsStaticBlocked(4, 4));
        Assert.False(adapter.IsStaticBlocked(3, 4));
        Assert.True(adapter.IsBlocked(4, 4));
        Assert.False(adapter.IsBlocked(2, 2));
        Assert.True(adapter.IsBlocked(-1, 2));
        Assert.True(adapter.IsBlocked(2, -1));
        Assert.True(adapter.IsBlocked(9, 2));
        Assert.True(adapter.IsBlocked(2, 9));
    }

    [Fact]
    public void DynamicObstacleCanMoveAcrossFramesAndBeRemoved()
    {
        var adapter = new JpsAdapter(12, 10, obstaclePadding: 1);
        Assert.True(adapter.UpdateDynamicObstacle(7, 3, 3, 2, 2));
        Assert.Equal(1, adapter.DynamicObstacleCount);
        Assert.True(adapter.Map.IsBlocked(2, 3));
        Assert.True(adapter.Map.IsBlocked(5, 4));

        int versionBeforeMove = adapter.Map.Version;
        Assert.True(adapter.UpdateDynamicObstacle(7, 4, 3, 2, 2));

        Assert.True(adapter.Map.IsWalkable(2, 3));
        Assert.True(adapter.Map.IsBlocked(3, 3));
        Assert.True(adapter.Map.IsBlocked(6, 4));
        Assert.Equal(8, adapter.Map.Version - versionBeforeMove); // only the entering/leaving strips changed

        Assert.True(adapter.UpdateDynamicObstacle(7, 0, 0, 0, 0));
        Assert.Equal(0, adapter.DynamicObstacleCount);
        Assert.True(adapter.Map.IsWalkable(3, 3));
        Assert.False(adapter.UpdateDynamicObstacle(7, 0, 0, 0, 0));
    }

    [Fact]
    public void RemovingDynamicSourcesPreservesStaticAndOverlappingCoverage()
    {
        var source = new GridMap(10, 10);
        source.SetBlocked(4, 4, true);
        var adapter = new JpsAdapter(source, obstaclePadding: 1);
        adapter.UpdateDynamicObstacle(1, 4, 4, 2, 1);
        adapter.UpdateDynamicObstacle(2, 5, 4, 1, 1);

        adapter.UpdateDynamicObstacle(1, 0, 0, 0, 0);
        Assert.True(adapter.Map.IsBlocked(5, 4));
        Assert.True(adapter.TryGetDynamicObstacle(2, out DynamicObstacle remaining));
        Assert.Equal(5, remaining.X); // swap-remove kept the moved last entry addressable by id

        adapter.UpdateDynamicObstacle(2, 0, 0, 0, 0);
        Assert.True(adapter.Map.IsBlocked(5, 4)); // static obstacle padding still covers it
        Assert.Equal(0, adapter.DynamicCoveredCellCount);

        adapter.UpdateDynamicObstacle(3, 7, 7, 1, 1);
        Assert.True(adapter.ClearDynamicObstacles());
        Assert.True(adapter.Map.IsBlocked(5, 4));
        Assert.True(adapter.Map.IsWalkable(7, 7));
    }

    [Fact]
    public void ChangingPaddingRebuildsStaticAndDynamicFootprints()
    {
        var source = new GridMap(12, 12);
        source.SetBlocked(8, 8, true);
        var adapter = new JpsAdapter(source);
        adapter.UpdateDynamicObstacle(3, 5, 5, 1, 1);

        Assert.True(adapter.SetObstaclePadding(2));
        Assert.True(adapter.Map.IsBlocked(3, 3));
        Assert.True(adapter.Map.IsBlocked(6, 8));
        Assert.True(adapter.Map.IsBlocked(1, 6)); // map boundary safety band

        Assert.True(adapter.SetObstaclePadding(0));
        Assert.True(adapter.Map.IsWalkable(3, 3));
        Assert.True(adapter.Map.IsWalkable(6, 8));
        Assert.True(adapter.Map.IsBlocked(5, 5));
        Assert.True(adapter.Map.IsBlocked(8, 8));
    }

    [Fact]
    public void StaticMapIsSnapshottedAndImmutable()
    {
        var source = new GridMap(8, 8);
        source.SetBlocked(3, 3, true);
        var adapter = new JpsAdapter(source, obstaclePadding: 1);

        source.SetBlocked(3, 3, false);
        source.SetBlocked(6, 6, true);

        Assert.True(adapter.IsStaticBlocked(3, 3));
        Assert.False(adapter.IsStaticBlocked(6, 6));
        Assert.True(adapter.Map.IsBlocked(2, 2));
        Assert.True(adapter.Map.IsWalkable(6, 6));
    }

    [Fact]
    public void ExternalPathfinderUsesAdapterSystemAfterSync()
    {
        var source = new GridMap(9, 7);
        for (int y = 1; y <= 5; y++)
            if (y != 3)
                source.SetBlocked(4, y, true);

        var adapter = new JpsAdapter(source);
        var pathfinder = new JpsPathfinder();
        adapter.Sync();
        Assert.True(pathfinder.FindPath(adapter.System, (2, 3), (6, 3)).Success);

        adapter.SetObstaclePadding(1);
        adapter.Sync();
        Assert.False(pathfinder.FindPath(adapter.System, (2, 3), (6, 3)).Success);
    }

    [Fact]
    public void InvalidDynamicSizeIsRejected()
    {
        var adapter = new JpsAdapter(8, 8);
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            adapter.UpdateDynamicObstacle(1, 2, 2, 0, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            adapter.UpdateDynamicObstacle(1, 2, 2, -1, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            adapter.UpdateDynamicObstacle(1, short.MinValue - 1, 2, 1, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            adapter.UpdateDynamicObstacle(1, 2, short.MaxValue + 1, 1, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            adapter.UpdateDynamicObstacle(1, 2, 2, ushort.MaxValue + 1, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            adapter.UpdateDynamicObstacle(1, 2, 2, 1, ushort.MaxValue + 1));
    }

    [Fact]
    public void PackedDynamicObstacleRoundTripsBoundaryValues()
    {
        Assert.Equal(12, Marshal.SizeOf<DynamicObstacle>());

        var adapter = new JpsAdapter(8, 8);
        Assert.True(adapter.UpdateDynamicObstacle(
            42, short.MinValue, short.MaxValue, ushort.MaxValue, ushort.MaxValue));

        Assert.True(adapter.TryGetDynamicObstacle(42, out DynamicObstacle obstacle));
        Assert.Equal(42, obstacle.Id);
        Assert.Equal(short.MinValue, obstacle.X);
        Assert.Equal(short.MaxValue, obstacle.Y);
        Assert.Equal(ushort.MaxValue, obstacle.Width);
        Assert.Equal(ushort.MaxValue, obstacle.Height);
    }

    [Fact]
    public void RandomUpdatesMatchAReferenceOccupancyModel()
    {
        const int width = 16, height = 13;
        var source = new GridMap(width, height);
        var staticBlocked = new bool[width, height];
        var dynamicObstacles = new Dictionary<int, (int X, int Y, int W, int H)>();
        var random = new Random(20260717);
        int padding = 0;

        for (int y = 0; y < height; y++)
            for (int x = 0; x < width; x++)
                if (random.NextDouble() < 0.12)
                {
                    source.SetBlocked(x, y, true);
                    staticBlocked[x, y] = true;
                }

        var adapter = new JpsAdapter(source);

        for (int step = 0; step < 300; step++)
        {
            int action = random.Next(3);
            if (action == 0)
            {
                int id = random.Next(6);
                int x = random.Next(-3, width + 3);
                int y = random.Next(-3, height + 3);
                int w = random.Next(1, 5), h = random.Next(1, 5);
                adapter.UpdateDynamicObstacle(id, x, y, w, h);
                dynamicObstacles[id] = (x, y, w, h);
            }
            else if (action == 1)
            {
                int id = random.Next(6);
                adapter.UpdateDynamicObstacle(id, 0, 0, 0, 0);
                dynamicObstacles.Remove(id);
            }
            else
            {
                padding = random.Next(4);
                adapter.SetObstaclePadding(padding);
            }

            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    bool expected = x < padding || x >= width - padding ||
                                    y < padding || y >= height - padding;

                    for (int sy = 0; sy < height && !expected; sy++)
                        for (int sx = 0; sx < width && !expected; sx++)
                            expected = staticBlocked[sx, sy] &&
                                       x >= sx - padding && x <= sx + padding &&
                                       y >= sy - padding && y <= sy + padding;

                    foreach (var obstacle in dynamicObstacles.Values)
                    {
                        if (x >= obstacle.X - padding &&
                            x <= obstacle.X + obstacle.W - 1 + padding &&
                            y >= obstacle.Y - padding &&
                            y <= obstacle.Y + obstacle.H - 1 + padding)
                        {
                            expected = true;
                            break;
                        }
                    }

                    Assert.Equal(expected, adapter.Map.IsBlocked(x, y));
                }
            }
        }
    }
}
