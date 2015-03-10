# ur5-microservice

FlexiMon UR5 Microservice

Local installation with:

<pre>
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
</pre>

Dependencies:

* rs{b|c|t}-0.11
* librstconverters-humavips0.11

These can be installed via binary packages from CoR-Lab's Debian package server.

Preconditions:

The rst converter library plugin(s) need to be configured to execute the examples. Exemplary rsb configuration entry:

<pre>
[plugins.cpp]
path = /usr/local/lib/rsb0.11/plugins
load = rsbrstconvertersstable
</pre>

Installed executables are (currently):

* rsb-ur5-microservice (provides the actual microservices)
* rsb-ur5-microservice-client (minimal test client)

