# AMR

.

## Setting up docker container

On remote machine
```
bash build_docker.sh
bash start_docker.sh
```

Inside docker
```
cd /root/amrl_msgs
make -j
cd /root/cs393_starter
make -j
cd /root/ut_automata
make -j
```

# Running the code
Then use tmux to create another window to start roscore. Use tmux to create two more windows to start
the simulator using `./bin/simulator` and `./bin/websocket`. Finally, create another window run the web 
browser by running `python3 -m http.server 8000`

Note, you should do these three commands in the /root/ut_automata directory.

# Connecting Locally

If you want to connect to the simulator locally, you need to forward the PORT that the files are being served to.
You can run the following command to do this:

```
ssh -L YOUR_LOCAL_PORT:0.0.0.0:8000 <user>@<remote machine>
```

Then you can connect to the simulator by going to http://localhost:YOUR_LOCAL_PORT in your browser.

# Connecting to Simulator

Set the robot IP address to the IP address that your docker container is running on. For robolidar, this is
10.0.0.211. 
