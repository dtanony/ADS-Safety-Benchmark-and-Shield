import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from uturn.uturn import *

def simulate_data(ve,vo,dx0,rightmost_lane):
    """
    :param vo: NPC speed in m/s
    :param ve: Ego speed in m/s
    :param dx0: Initial longitudinal distance in m
    :param rightmost_lane: True if ego on rightmost lane, False if ego on the adjacent lane
    """
    sim_step = 0.02
    average_length = (ego_length + npc_length) / 2

    npc = UTurnNPC((dx0 + average_length, 0),
                   (vo, 0),
                   (npc_length, npc_width),
                   WHEEL_BASE, TURNING_WHEEL_ANGLE)

    ego_dy0 = MEDIAN_STRIP + LANE_WIDTH
    if not rightmost_lane:
        ego_dy0 += LANE_WIDTH
    ego = UTurnEgo((0, ego_dy0),
                   (ve, 0),
                   (ego_length, ego_width))

    sim = UTurnSimulation(ego, npc, sim_step)
    ego_positions = []
    npc_positions = []
    ego_vertices = []
    npc_vertices = []

    while sim.time < 15 and not sim.collision:
        ego_positions.append(np.copy(sim.ego.position))
        npc_positions.append(np.copy(sim.npc.position))
        ego_vertices.append(np.copy(sim.ego.get_vertices()))
        npc_vertices.append(np.copy(sim.npc.get_vertices()))
        sim.step()

    return ego_positions, npc_positions, ego_vertices, npc_vertices

def visualize(ve,vo,dx0,rightmost_lane):
    """
    :param vo: NPC speed in m/s
    :param ve: Ego speed in m/s
    :param dx0: Initial longitudinal distance in m
    :param rightmost_lane: True if ego on rightmost lane, False if ego on the adjacent lane
    """
    veh1_positions, veh2_positions, veh1_vertices, veh2_vertices = (
        simulate_data(ve,vo,dx0,rightmost_lane))
    veh1_positions = np.array(veh1_positions)
    veh2_positions = np.array(veh2_positions)
    veh1_vertices = np.array(veh1_vertices)
    veh2_vertices = np.array(veh2_vertices)
    frames_count = len(veh1_positions)

    fig, ax = plt.subplots(figsize=(16, 8))
    plt.subplots_adjust(bottom=0.05)  # space for buttons
    ax.set_xlim(-2, 45)
    ax.set_ylim(-2, 12)
    ax.set_aspect('equal')

    ax.axhline(y=2.75, color='gray', linestyle='--', linewidth=1)
    ax.axhline(y=6.25, color='gray', linestyle='--', linewidth=1)
    ax.axhline(y=9.75, color='gray', linestyle='--', linewidth=1)

    veh1_patch = Polygon(veh1_vertices[0], closed=True, color='blue', alpha=0.5)
    veh2_patch = Polygon(veh2_vertices[0], closed=True, color='red', alpha=0.5)
    ax.add_patch(veh1_patch)
    ax.add_patch(veh2_patch)

    veh1_center, = ax.plot([], [], 'bo')
    veh2_center, = ax.plot([], [], 'ro')

    frame_text = ax.text(0.02, 0.9, '', transform=ax.transAxes, fontsize=10,
                         bbox=dict(facecolor='white', alpha=0.7))
    info_text = ax.text(0.35, 0.9, '', transform=ax.transAxes, fontsize=12)
    info_text.set_text(f"ve={int(ve * 3.6)}, vo={int(vo * 3.6)}, dx0={dx0}, "
                       f'Ego: {"rightmost lane" if rightmost_lane else "adjacent lane"}')

    # Animation state
    anim_running = True
    current_frame = [0]

    def update(frame):
        # Delay animation 4s
        # if i < 100:
        #     frame = 0
        # else:
        #     frame = i - 100
        current_frame[0] = frame
        veh1_patch.set_xy(veh1_vertices[frame])
        veh2_patch.set_xy(veh2_vertices[frame])
        veh1_center.set_data([veh1_positions[frame, 0]], [veh1_positions[frame, 1]])
        veh2_center.set_data([veh2_positions[frame, 0]], [veh2_positions[frame, 1]])
        frame_text.set_text(f"Frame: {frame}/{frames_count-1}")
        return veh1_patch, veh2_patch, veh1_center, veh2_center, frame_text

    ani = FuncAnimation(fig, update, frames=frames_count, interval=40, blit=True, repeat=False)

    # Play/Pause button
    ax_play = plt.axes([0.7, 0.05, 0.1, 0.075])
    btn_play = Button(ax_play, 'Pause')

    def toggle_play(event):
        nonlocal anim_running
        if anim_running:
            ani.event_source.stop()
            btn_play.label.set_text("Play")
        else:
            ani.event_source.start()
            btn_play.label.set_text("Pause")
        anim_running = not anim_running

    btn_play.on_clicked(toggle_play)

    plt.show()


if __name__ == '__main__':
    parser = cli_parser()
    parser.add_argument('-dx0', type=int, default=25,
                        help='Initial distance dx0 in m (default: 25)')
    parser.add_argument('-ve', type=int, default=20,
                      help='AV Speed in km/h (default: 20)')
    cli_args = parser.parse_args()

    ve = cli_args.ve / 3.6
    vo = cli_args.vo / 3.6
    dx0 = cli_args.dx0
    rightmost = cli_args.lane == "rightmost"
    if cli_args.lane not in ["rightmost", "adjacent"]:
        print("[WARNING] Lane must be either `rightmost` or `adjacent`. "
              "Rightmost is used by default")
        rightmost = True
    visualize(ve,vo,dx0,rightmost)