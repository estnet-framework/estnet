**Linking with INET and ESTNeT**

Define environment variables to be able to include INET and ESTNeT lib:
- `export INET_ROOT="Path_to_your_local_INET_directory"` <br>
- `export ESTNET_ROOT="Path_to_your_local_ESTNeT_directory"` <br>
Example path: `/C/Users/my_user/workspace_omnetpp/inet`

The environment variables can also be set automatically. When you use the mingw console in Windows you can do that by the following commands:
- `cd ~`
- `echo 'export INET_ROOT="Path_to_your_local_INET_directory"' >> .bash_profile`<br>
- `echo 'export ESTNET_ROOT="Path_to_your_local_ESTNeT_directory"' >> .bash_profile`<br>
Restart the MinGW Shell before running run_sim.sh

**Running a simulation on the command-line**

- Run mingwenv.cmd in the OMNeT++ directory
- Go to the folder simulations in the template project
- `./run_sim.sh <build config> <config file> [<config name>] [<Cmdenv|Qtenv>] [extra omnet args ...]` <br>
(Example: `./run_sim.sh release omnetpp.ini`)
