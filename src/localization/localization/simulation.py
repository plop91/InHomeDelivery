from particle_filter import ParticleFilter


def main(pf):
    pf = pf
    # set robots initial pose
    # while True:

    # generate odometry info
    # format = [[lx,ly,lh], [cx, cy, ch]]
    odom = [[0, 0, 0], [0, 0, 0]]

    # generate marker poses
    # format [[x,y,h], ...]
    marker_poses = []

    # run particle filter
    x, y, h, confidence = pf.update(odom, marker_poses)

    print(f"x:{x} y:{y} confidence:{confidence}")


if __name__ == "__main__":
    f = "maps/new_map.json"
    p = ParticleFilter(f)
    main(p)
