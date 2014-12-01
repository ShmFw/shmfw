shmfw_oszi

Have a look on the oszi_example.cpp it gernerates a sinus wave :-)

This model plots a ShmFw::Vector<double> entries into a window like an oscilloscope.
Workflow:
Add to your program a ShmFw::Vector<double> variable, something like:
ShmFw::Vector<double> a ( "data", shmHdl);
Every vector entry represents a channel to print.
It is important to trigger a change with ShmFw::Vector<double>::itHasChanged(), locking is optional.

Now after you started you program to log you can view the data using the shmfs-oszi program.
You can use the option -v [ --variable_name ] to define which ShmFw::Vector<double> should be printed. 
If you have problems or if you messed up your shared memory you can clear it by using the 
shmfs-admin with the -c [ --clear ] option.

The shmfs-oszi program  generates a configuration file oszi.cfg you can edit this file to 
ajust the visualized legend colors, timing, trigger … as you like

Greetings
Markus
