/* ============================================================
 *
 * This file is a part of the ur5-microservice project
 *
 * Copyright (C) 2015 by Sebastian Wrede <swrede at techfak dot uni-bielefeld dot de>
 *
 * This file may be licensed under the terms of the
 * GNU Lesser General Public License Version 3 (the ``LGPL''),
 * or (at your option) any later version.
 *
 * Software distributed under the License is distributed
 * on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 * express or implied. See the LGPL for the specific language
 * governing rights and limitations.
 *
 * You should have received a copy of the LGPL along with this
 * program. If not, go to http://www.gnu.org/licenses/lgpl.html
 * or write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The development of this software was supported by:
 *   CoR-Lab, Research Institute for Cognition and Robotics
 *     Bielefeld University
 *
 * ============================================================ */

// mark-start::show-for-parsing
#include <iostream>

#include <stdlib.h>
#include <math.h>

#include <boost/program_options.hpp>

#include <rsb/Informer.h>
#include <rsb/Factory.h>

#include <rsc/misc/SignalWaiter.h>

#include "library/ur5-rsb-adapter.h"

//ROS
#include <ros/ros.h>

using namespace std;
using namespace rsb;
using namespace ur5_microservice::library;
namespace po = boost::program_options;

string scope = "/fleximon/module1/ur5";
int value = 0;

void handleCommandline(int argc, char *argv[]) {

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")("scope,s",
            po::value < string > (&scope), "Scope to output the result on");
          //  "value,v", po::value<int>(&value)->required(),
          //  "The value to process.");

    // allow to give the value as a positional argument
    po::positional_options_description p;
    p.add("value", 1);

    po::variables_map vm;
    po::store(
            po::command_line_parser(argc, argv).options(options).positional(p).run(),
            vm);

    // first, process the help option
    if (vm.count("help")) {
        cout << options << "\n";
        exit(1);
    }

    // afterwards, let program options handle argument errors
    po::notify(vm);

    // you can do some additional validity checks here

    // print args
    cout << "Running on scope: " << scope << endl;

}

int main(int argc, char *argv[]) {

	rsc::misc::initSignalWaiter();

    // ROS Init
    ros::init(argc, argv, "ur5_manipulation");

	// program options
    handleCommandline(argc, argv);

    // rsb microservice endpoints
    boost::shared_ptr<UR5RSBAdapter> adapter = boost::shared_ptr<UR5RSBAdapter>(new UR5RSBAdapter(scope));


    // wait here so incoming method calls can be processed.
    return rsc::misc::suggestedExitCode(rsc::misc::waitForSignal());



}
// mark-end::show-for-parsing
