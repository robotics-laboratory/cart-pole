from cartpole import State, LogServer
import time



def main():
    server = LogServer(log_path='log.mcap')

    for i in range(1000):
        time.sleep(0.1)
        server.publish(State(pole_angle=i/3.14/10))


if __name__ == '__main__':
    main()
