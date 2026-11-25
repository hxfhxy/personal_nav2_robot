#!/usr/bin/env python3
"""Generate maps/playground.{pgm,yaml} from the known playground.sdf obstacles."""

# 不跑 SLAM，直接按世界文件里的方块和墙栅格化出一张「真值地图」。
# 输出的 PGM 按 map_server 的约定读：0 = 障碍，254 = 可走，205 = 未知。
# 行从上往下对应 y 递减，所以算 y 要用 ymax 往上减。

from pathlib import Path


def fill_rect(grid, xmin, ymin, res, cx, cy, sx, sy):
    """把以 (cx, cy) 为中心、sx × sy 的矩形涂成障碍（0）。

    逐格判断中心点是否落在矩形里，不做抗锯齿：0.05 m 的格子比方块小得多，够用。
    """
    height = len(grid)
    width = len(grid[0])
    x0, x1 = cx - sx / 2.0, cx + sx / 2.0
    y0, y1 = cy - sy / 2.0, cy + sy / 2.0
    ymax = ymin + height * res
    for row in range(height):
        y = ymax - (row + 0.5) * res
        if y < y0 or y > y1:
            continue
        for col in range(width):
            x = xmin + (col + 0.5) * res
            if x0 <= x <= x1:
                grid[row][col] = 0


def main():
    pkg = Path(__file__).resolve().parents[1]
    maps_dir = pkg / 'maps'
    maps_dir.mkdir(parents=True, exist_ok=True)

    res = 0.05
    xmin, xmax = -5.0, 5.0
    ymin, ymax = -5.0, 5.0
    width = int(round((xmax - xmin) / res))
    height = int(round((ymax - ymin) / res))
    grid = [[254 for _ in range(width)] for _ in range(height)]  # 整张图先涂成可走

    # 四个 0.5 m 方块 + y = 3 附近一堵 4 m 长的墙，和 playground.sdf 对齐
    fill_rect(grid, xmin, ymin, res, 2.0, 0.0, 0.5, 0.5)
    fill_rect(grid, xmin, ymin, res, 2.0, 2.0, 0.5, 0.5)
    fill_rect(grid, xmin, ymin, res, -2.0, 2.0, 0.5, 0.5)
    fill_rect(grid, xmin, ymin, res, -2.0, -2.0, 0.5, 0.5)
    fill_rect(grid, xmin, ymin, res, 0.0, 3.0, 4.0, 0.2)

    pgm = maps_dir / 'playground.pgm'
    with pgm.open('wb') as handle:
        handle.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
        for row in grid:
            handle.write(bytes(row))

    yaml = maps_dir / 'playground.yaml'
    yaml.write_text(
        'image: playground.pgm\n'
        'mode: trinary\n'
        f'resolution: {res}\n'
        f'origin: [{xmin}, {ymin}, 0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n'
    )
    print(f'Wrote {pgm} and {yaml}')


if __name__ == '__main__':
    main()
