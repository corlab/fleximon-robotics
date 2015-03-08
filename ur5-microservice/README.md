# ur5-microservice

FlexiMon UR5 Microservice

Local installation with:

<pre>
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
</pre>

Dependencies:

* rs{b|c|t}-0.11

Preconditions:

The rst converter library plugin(s) need to be configured to execute the examples. Exemplary entry in <pre>rsb.conf</pre>:

<pre>
[plugins.cpp]
path = /usr/local/lib/rsb0.11/plugins
load = rsbrstconvertersstable
</pre>
