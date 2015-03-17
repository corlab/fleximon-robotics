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

#include "ur5-rsb-adapter.h"

#include <rsb/Factory.h>
#include <rst/geometry/Pose.pb.h>
#include <rst/geometry/Rotation.pb.h>
#include <rst/geometry/Translation.pb.h>
#include <rst/kinematics/JointAngles.pb.h>

#include <iostream>
#include <string>

using namespace std;
using namespace rsb;
using namespace rsb::patterns;

namespace ur5_microservice {
namespace library {

/**
 * Move in Cartesian space towards a desired target position using
 * planning+IK features of respective backend.
 */

class MoveToCallback: public LocalServer::Callback<rst::geometry::Pose, void> {
public:
    void call(const std::string& /*methodName*/,
                                        boost::shared_ptr<rst::geometry::Pose> input) {
    	std::cout << "MoveCS method called with rst::geometry::pose: Translation (x,y,z): " << input->translation().x() << "," << input->translation().y() << "," << input->translation().z() << " ";
    	std::cout << " Rotation (qx,qy,qz,qw): " << input->rotation().qx() << "," << input->rotation().qy() << "," << input->rotation().qz() << "," << input->rotation().qw() << std::endl;
    }
};

/**
 * Move in joint space directly to a particular robot pose.
 */
class MoveCallback: public LocalServer::Callback<rst::kinematics::JointAngles, void> {
    void call(const std::string& /*methodName*/,
    		  boost::shared_ptr<rst::kinematics::JointAngles> input) {
        std::cout << "MoveJS method called with rst:kinematics:JointAngles vector in rad: ";
        for (int i=0; i<input->angles_size(); i++) {
        	std::cout << " " << input->angles(i) << " ";
        }
        std::cout << std::endl;
    }
};

UR5RSBAdapter::UR5RSBAdapter(const std::string& scope) {
    // Use the RSB factory to create a Server instance that provides
    // callable methods under the scope /example/server.
    Factory& factory = getFactory();
    server = factory.createLocalServer(scope);

    // Register method with name and implementing callback object.
    server->registerMethod("moveRobot", LocalServer::CallbackPtr(new MoveCallback()));
    server->registerMethod("moveToRobot", LocalServer::CallbackPtr(new MoveToCallback()));
}

UR5RSBAdapter::~UR5RSBAdapter() {
}

/**
 * just used here as an example for GTest
 */
int UR5RSBAdapter::transform(const int& magicNumber) {
    return magicNumber * 2;
}

}
}
