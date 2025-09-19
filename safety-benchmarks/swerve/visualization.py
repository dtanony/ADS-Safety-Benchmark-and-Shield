import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from swerve.swerve import *

def simulate_data(ve,vo,dx0,vy):
    """
    :param vo: NPC speed in m/s
    :param ve: Ego speed in m/s
    :param dx0: Initial longitudinal distance in m
    :param vy: Average lateral velocity in m/s
    """
    sim_step = 0.02
    average_length = (ego_length + npc_length) / 2.0

    npc = SwerveNPC((dx0 + average_length, 0.0),
                    vo, vy,
                    (npc_length, npc_width),
                    NY, SWERVE_DISTANCE)

    ego = SwerveEgo((0.0, LANE_WIDTH),
                    (ve, 0.0),
                    (ego_length, ego_width))

    sim = SwerveSimulation(ego, npc, sim_step)
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

    return (ego_positions, npc_positions, ego_vertices, npc_vertices,
            npc.waypoints)

def visualize(ve,vo,dx0,vy):
    """
    :param vo: NPC speed in m/s
    :param ve: Ego speed in m/s
    :param dx0: Initial longitudinal distance in m
    :param vy: Average lateral velocity in m/s
    """
    (veh1_positions, veh2_positions, veh1_vertices, veh2_vertices, waypoints) = (
        simulate_data(ve,vo,dx0,vy))
    veh1_positions = np.array(veh1_positions)
    veh2_positions = np.array(veh2_positions)
    veh1_vertices = np.array(veh1_vertices)
    veh2_vertices = np.array(veh2_vertices)
    frames_count = len(veh1_positions)

    fig, ax = plt.subplots(figsize=(16, 8))
    plt.subplots_adjust(bottom=0.05)  # space for buttons
    ax.set_xlim(-2, 55)
    ax.set_ylim(-2, 10)
    ax.set_aspect('equal')

    ax.axhline(y=LANE_WIDTH/2, color='gray', linestyle='--', linewidth=1)
    ax.axhline(y=LANE_WIDTH*1.5, color='gray', linestyle='--', linewidth=1)

    veh1_patch = Polygon(veh1_vertices[0], closed=True, color='blue', alpha=0.5)
    veh2_patch = Polygon(veh2_vertices[0], closed=True, color='red', alpha=0.5)
    ax.add_patch(veh1_patch)
    ax.add_patch(veh2_patch)

    veh1_center, = ax.plot([], [], 'bo', markersize=3)
    veh2_center, = ax.plot([], [], 'ro', markersize=3)

    frame_text = ax.text(0.02, 0.9, '', transform=ax.transAxes, fontsize=10,
                         bbox=dict(facecolor='white', alpha=0.7))
    info_text = ax.text(0.35, 0.9, '', transform=ax.transAxes, fontsize=12)
    info_text.set_text(f"ve={int(ve*3.6)}, vo={int(vo*3.6)}, dx0={dx0}, vy={vy}")

    for waypoint in waypoints:
        ax.plot(waypoint[0], waypoint[1], 'go', markersize=3)

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
    parser.add_argument('-vy', type=float, default=1.2,
                        help='Average lateral velocity vy in m/s (default: 1.2)')
    cli_args = parser.parse_args()

    ve = cli_args.ve / 3.6
    vo = cli_args.vo / 3.6
    dx0 = cli_args.dx0
    vy = cli_args.vy
    visualize(ve,vo,dx0,vy)