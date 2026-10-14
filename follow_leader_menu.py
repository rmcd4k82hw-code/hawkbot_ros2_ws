"""
Follow-the-Leader Control Menu
================================
Terminal menu to start/stop the Follow-the-Leader mode
where Robot2 follows Robot1's blue trail in real-time.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys


class FollowLeaderMenuClient(Node):
    def __init__(self):
        super().__init__('follow_leader_menu_client')

        # Follow Leader services (runs under robot2 namespace by default)
        self.cli_start = self.create_client(Trigger, '/robot2/follow_leader/start')
        self.cli_stop = self.create_client(Trigger, '/robot2/follow_leader/stop')

        # Trail recording for robot1 (manual controls)
        self.cli_trail_start = self.create_client(Trigger, '/robot1/trajectory_recorder/start')
        self.cli_trail_stop = self.create_client(Trigger, '/robot1/trajectory_recorder/stop_and_save')
        self.cli_trail_clear = self.create_client(Trigger, '/robot1/trajectory_recorder/clear')

    def call_service(self, client, name):
        if not client.wait_for_service(timeout_sec=3.0):
            print(f"\n[ERREUR] Le service '{name}' n'est pas disponible. "
                  f"Assurez-vous que la navigation est lancée !\n")
            return

        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            res = future.result()
            symbol = "✓" if res.success else "✗"
            print(f"\n[{symbol}] {res.message}\n")
        else:
            print(f"\n[ERREUR] Échec de l'appel au service '{name}'\n")


def main(args=None):
    rclpy.init(args=args)
    client = FollowLeaderMenuClient()

    while rclpy.ok():
        print("=" * 65)
        print("   HAWKBOT - MODE FOLLOW-THE-LEADER (Robot2 follows Robot1)")
        print("=" * 65)
        print("  1. START THE MODE FOLLOW-THE-LEADER")
        print("     (START THE BLUE TRAIL ROBOT1 AND FOLLOWING OF THE ROBOT2)")
        print("  2. STOP THE MODE FOLLOW-THE-LEADER")
        print("     (STOP FOLLOWING AND SAVE TRAJECTORY)")
        print("  -----------------------------------------------------------")
        print("  3. START MANUALY THE BLUE TRAIL ROBOT1")
        print("  4. STOP AND SAVE THE BLUE TRAIL")
        print("  5. REMOVE THE BLUE TRAIL ROBOT1")
        print("  6. QUIT")
        print("=" * 65)

        choice = input("CHOOSE AN OPTION [1-6] : ").strip()

        if choice == '1':
            client.call_service(client.cli_start, "Follow Leader Start")
        elif choice == '2':
            client.call_service(client.cli_stop, "Follow Leader Stop")
        elif choice == '3':
            client.call_service(client.cli_trail_start, "Trail Start (Robot1)")
        elif choice == '4':
            client.call_service(client.cli_trail_stop, "Trail Stop & Save (Robot1)")
        elif choice == '5':
            client.call_service(client.cli_trail_clear, "Trail Clear (Robot1)")
        elif choice == '6':
            print("\nFermeture du menu Follow-the-Leader. Au revoir !\n")
            break
        else:
            print("\n[!] Choix invalide. Veuillez saisir un nombre entre 1 et 6.\n")

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
