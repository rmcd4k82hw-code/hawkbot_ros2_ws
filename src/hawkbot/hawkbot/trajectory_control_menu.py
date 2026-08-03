import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys

class TrajectoryControlClient(Node):
    def __init__(self, robot_ns='robot1'):
        super().__init__('trajectory_control_client')
        self.robot_ns = robot_ns
        
        self.cli_start = self.create_client(Trigger, f'/{self.robot_ns}/trajectory_recorder/start')
        self.cli_stop = self.create_client(Trigger, f'/{self.robot_ns}/trajectory_recorder/stop_and_save')
        self.cli_clear = self.create_client(Trigger, f'/{self.robot_ns}/trajectory_recorder/clear')
        
        # Route Follower Clients for Robot 1 and Robot 2
        self.cli_rf1_start = self.create_client(Trigger, '/robot1/route_follower/start')
        self.cli_rf1_stop = self.create_client(Trigger, '/robot1/route_follower/stop')
        self.cli_rf2_start = self.create_client(Trigger, '/robot2/route_follower/start')
        self.cli_rf2_stop = self.create_client(Trigger, '/robot2/route_follower/stop')

    def call_service(self, client, name):
        if not client.wait_for_service(timeout_sec=3.0):
            print(f"\n[ERREUR] Le service '{name}' n'est pas disponible. Assurez-vous que la navigation est lancée !\n")
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
    
    robot_ns = 'robot1'
    if len(sys.argv) > 1:
        robot_ns = sys.argv[1]
        
    client = TrajectoryControlClient(robot_ns=robot_ns)

    while rclpy.ok():
        print("=" * 65)
        print("   HAWKBOT - CONTRÔLEUR DE TRAJECTOIRE BLEUE ET SUIVI DE PARCOURS")
        print("=" * 65)
        print("  1. Démarrer l'enregistrement de la traînée bleue")
        print("  2. Arrêter, fermer la boucle et sauvegarder le parcours")
        print("  3. Effacer la traînée bleue")
        print("  -----------------------------------------------------------")
        print("  4. Lancer le suivi du parcours bleu pour ROBOT 1")
        print("  5. Lancer le suivi du parcours bleu pour ROBOT 2")
        print("  6. Stopper le suivi de parcours (Robot 1 et Robot 2)")
        print("  7. Quitter")
        print("=" * 65)
        
        choice = input("Choisissez une option [1-7] : ").strip()

        if choice == '1':
            client.call_service(client.cli_start, "Start Recording")
        elif choice == '2':
            client.call_service(client.cli_stop, "Stop & Save")
        elif choice == '3':
            client.call_service(client.cli_clear, "Clear Trail")
        elif choice == '4':
            client.call_service(client.cli_rf1_start, "Start Following (Robot 1)")
        elif choice == '5':
            client.call_service(client.cli_rf2_start, "Start Following (Robot 2)")
        elif choice == '6':
            client.call_service(client.cli_rf1_stop, "Stop Following (Robot 1)")
            client.call_service(client.cli_rf2_stop, "Stop Following (Robot 2)")
        elif choice == '7':
            print("\nFermeture du menu de contrôle. Au revoir !\n")
            break
        else:
            print("\n[!] Choix invalide. Veuillez saisir un nombre entre 1 et 7.\n")

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
