Note: If you did not set the required environment variables for running a simulation from the command-line yet, please refer to the [command-line simulation wiki page](Running-a-simulation-from-the-command-line).

# Full Contact Plan Generation
1.  Switch the current directory to the `contactplan` folder in the [template project](https://github.com/estnet-framework/estnet-template)
2.  Run `./createContactPlan.sh <build config> <config file> <config name> <time limit in seconds>`. Example: `./createContactPlan.sh release omnetpp.ini General 10000s`
3. The contact plan will be written to `../simulations/results/contact_plan-<config name>-0.txt`
