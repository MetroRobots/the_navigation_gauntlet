import argparse
from matplotlib.pyplot import subplots, show
import pathlib
from math import hypot, atan2

from navigation_metrics import FlexibleBag, global_metric_search


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bagfile', type=pathlib.Path)
    args = parser.parse_args()

    global_metric_search()

    bag = FlexibleBag(args.bagfile, write_mods=False)

    t0 = bag.get_start_time()
    fig, ax = subplots(1)
    xs = []
    sizes = []
    colors = []
    scale = 30
    for t, msg in bag['/cmd_vel']:
        dt = t - t0
        x = msg.linear.x
        yaw = msg.angular.z
        xs.append(dt)
        colors.append(yaw)
        M = scale * max(x, 0.03) / 0.5
        sizes.append(M*M)

    ax.scatter(xs, [0.0] * len(xs), s=sizes, marker='|',
               c=colors, cmap='twilight_shifted')

    pxs = []
    pys = []
    psizes = []
    pcolors = []
    for ped_rmsg, path_rmsg in bag['/pedestrians', '/path2d']:
        p0 = path_rmsg.msg.pose
        for pedestrian in ped_rmsg.msg.pedestrians:
            p1 = pedestrian.pose
            dx = p1.x - p0.x
            dy = p1.y - p0.y
            d = hypot(dx, dy)
            angle = atan2(dy, dx) - p0.theta
            pxs.append(ped_rmsg.t - t0)
            pys.append(angle)
            s = 10 / d
            psizes.append(s*s)
            pcolors.append(hypot(pedestrian.velocity.x, pedestrian.velocity.y))

    ax.scatter(pxs, pys, s=psizes, c=pcolors, cmap='summer')

    show()


if __name__ == '__main__':
    main()
