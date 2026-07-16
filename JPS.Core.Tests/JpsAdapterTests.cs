using JPS.Models;
using JPS.Pathfinding;
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
    public void RemovingOneSourcePreservesOverlappingCoverage()
    {
        var adapter = new JpsAdapter(10, 10);
        adapter.SetStaticBlocked(4, 4, true);
        adapter.UpdateDynamicObstacle(1, 4, 4, 2, 1);
        adapter.UpdateDynamicObstacle(2, 5, 4, 1, 1);

        adapter.SetStaticBlocked(4, 4, false);
        adapter.UpdateDynamicObstacle(1, 0, 0, 0, 0);
        Assert.True(adapter.Map.IsBlocked(5, 4));

        adapter.UpdateDynamicObstacle(2, 0, 0, 0, 0);
        Assert.True(adapter.Map.IsWalkable(5, 4));
    }

    [Fact]
    public void ChangingPaddingRebuildsStaticAndDynamicFootprints()
    {
        var adapter = new JpsAdapter(12, 12);
        adapter.SetStaticBlocked(8, 8, true);
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
    }

    [Fact]
    public void RandomUpdatesMatchAReferenceOccupancyModel()
    {
        const int width = 16, height = 13;
        var adapter = new JpsAdapter(width, height);
        var staticBlocked = new bool[width, height];
        var dynamicObstacles = new Dictionary<int, (int X, int Y, int W, int H)>();
        var random = new Random(20260717);
        int padding = 0;

        for (int step = 0; step < 300; step++)
        {
            int action = random.Next(4);
            if (action == 0)
            {
                int x = random.Next(width), y = random.Next(height);
                bool blocked = random.Next(2) != 0;
                adapter.SetStaticBlocked(x, y, blocked);
                staticBlocked[x, y] = blocked;
            }
            else if (action <= 2)
            {
                int id = random.Next(6);
                int x = random.Next(-3, width + 3);
                int y = random.Next(-3, height + 3);
                int w = random.Next(1, 5), h = random.Next(1, 5);
                adapter.UpdateDynamicObstacle(id, x, y, w, h);
                dynamicObstacles[id] = (x, y, w, h);
            }
            else if (random.Next(2) == 0)
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
