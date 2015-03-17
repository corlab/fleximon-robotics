# ur5-microservice

FlexiMon UR5 Microservice

Local installation with:

<pre>
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
</pre>

Dependencies:

* rs{b|c|t}0.11 plus converter plugin library

These can be installed with the following command via binary packages from CoR-Lab's Debian package server. Further install documentation is available here: http://docs.cor-lab.de//rsb-manual/0.11/html/install-binary.html#debian-packages

<pre>
sudo apt-get install rsb-tools-cpp0.11 spread librsbspread0.11  librstconverters-humavips0.11
</pre>

Execution:

Preconditions:

The rst converter library plugin(s) need to be configured to execute the examples. Exemplary rsb configuration entry:

<pre>
[plugins.cpp]
path = /usr/lib/rsb0.11/plugins
load = rsbrstconvertersstable
</pre>

Installed executables are (currently):

* rsb-ur5-microservice (provides the actual microservices)
* rsb-ur5-microservice-client (minimal test client)

Network RSB Logger:
<pre>
rsb-loggercl0.11 -s monitor/scope socket:
rsb-introspectcl0.11 -s monitor/object-tree socket:
</pre>
<pre>
sudo apt-get install rsb-tools-cl0.11
</pre>
