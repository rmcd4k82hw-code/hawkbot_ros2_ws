import sys
import socket
import argparse
import signal

def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description="Lidar UDP demultiplexer proxy for multi-robot bringup")
    parser.add_argument('--ip1', type=str, default='', help='IP of robot 1')
    parser.add_argument('--ip2', type=str, default='', help='IP of robot 2')
    parser.add_argument('--ip3', type=str, default='', help='IP of robot 3')
    parser.add_argument('--ip4', type=str, default='', help='IP of robot 4')
    args, unknown = parser.parse_known_args()

    # Build IP to port map
    # Offsets: robot1=10 (18912), robot2=100 (19002), robot3=200 (19102), robot4=300 (19202)
    ip_to_port = {}
    if args.ip1: ip_to_port[args.ip1] = 18912
    if args.ip2: ip_to_port[args.ip2] = 19002
    if args.ip3: ip_to_port[args.ip3] = 19102
    if args.ip4: ip_to_port[args.ip4] = 19202

    if not ip_to_port:
        print("[lidar_demux] No robot IPs specified. Exiting.", flush=True)
        return

    print(f"[lidar_demux] Starting UDP Demux Proxy on port 18902.", flush=True)
    print(f"[lidar_demux] Forwarding mappings:", flush=True)
    for ip, port in ip_to_port.items():
        print(f"  - {ip} -> 127.0.0.1:{port}", flush=True)

    # Create socket
    sock_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock_in.bind(('0.0.0.0', 18902))
    except Exception as e:
        print(f"[lidar_demux] Failed to bind to port 18902: {e}", flush=True)
        sock_in.close()
        sys.exit(1)

    sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Handle shutdown signals cleanly
    running = True
    def signal_handler(sig, frame):
        nonlocal running
        print("[lidar_demux] Shutdown signal received. Exiting...", flush=True)
        running = False
        
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    sock_in.settimeout(0.5)
    packet_counts = {ip: 0 for ip in ip_to_port}

    try:
        while running:
            try:
                data, addr = sock_in.recvfrom(4096)
                sender_ip = addr[0]
                if sender_ip in ip_to_port:
                    target_port = ip_to_port[sender_ip]
                    sock_out.sendto(data, ('127.0.0.1', target_port))
                    packet_counts[sender_ip] += 1
                    
                    # Log progress every 100 packets to verify it is alive
                    if packet_counts[sender_ip] % 100 == 1:
                        print(f"[lidar_demux] Forwarded packet {packet_counts[sender_ip]} from {sender_ip} to 127.0.0.1:{target_port}", flush=True)
                else:
                    # Log unknown senders once in a while
                    print(f"[lidar_demux] Received packet from UNKNOWN sender {sender_ip}", flush=True)
            except socket.timeout:
                continue
            except Exception as e:
                if running:
                    print(f"[lidar_demux] Socket error: {e}", flush=True)
                break
    finally:
        sock_in.close()
        sock_out.close()
        print("[lidar_demux] Proxy stopped cleanly. Totals:", flush=True)
        for ip, count in packet_counts.items():
            print(f"  - {ip}: {count} packets", flush=True)

if __name__ == '__main__':
    main()
