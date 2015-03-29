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
#include <stdlib.h>

#include <rsb/Factory.h>
#include <rst/geometry/Pose.pb.h>
#include <rst/geometry/Rotation.pb.h>
#include <rst/geometry/Translation.pb.h>
#include <rst/kinematics/JointAngles.pb.h>

using namespace rsb;
using namespace rst;
using namespace rsb::patterns;

int main(int /*argc*/, char** /*argv*/) {

    // Use the RSB factory to create a RemoteServer instance for the
    // server at scope /example/server.
    Factory& factory = getFactory();
    RemoteServerPtr remoteServer
        = factory.createRemoteServer("/fleximon/module1/ur5");

    // Call the method "echo", passing it a string value as argument
    // and accepting a string value as result. Note that the types of
    // arguments and return values are defined by the server providing
    // the respective methods and have to be matched in method calls.
    boost::shared_ptr<rst::kinematics::JointAngles> jointAngles(new rst::kinematics::JointAngles);
    jointAngles->add_angles( 0.785);
    jointAngles->add_angles(-1.57);
    jointAngles->add_angles(1.57);
    jointAngles->add_angles(-1.57);
    jointAngles->add_angles(-1.57);
    jointAngles->add_angles(0.785);

//    remoteServer->call<std::string>("moverobot", jointAngles);
//    std::cout << "Called move!" << std::endl;

    //std::cout << "Server replied: " << *result << std::endl;

    // Call the method "echo" without waiting for the call to return a
    // result: instead of a result, a "future" object is returned,
    // from which the actual result can be obtained at a later point
    // in time. In this example, the future.get(10) call may block for
    // up to 10 seconds and throw an exception if a result is not
    // received within that time.
    // the respective methods and have to be matched in method calls.
    boost::shared_ptr<rst::geometry::Pose> pose(new rst::geometry::Pose());
    pose->mutable_translation()->set_x(0.400);
    pose->mutable_translation()->set_y(0.400);
    pose->mutable_translation()->set_z(0.400);
    pose->mutable_rotation()->set_qw(0);
    pose->mutable_rotation()->set_qx(0);
    pose->mutable_rotation()->set_qy(0);
    pose->mutable_rotation()->set_qz(0);

//    RemoteServer::DataFuture<std::string> future
 //       = remoteServer->callAsync<std::string>("movetorobot", pose);
  //  std::cout << "Called moveToRobot" << std::endl;

    // We could do something else here while the server processes the
    // call.

    pose = remoteServer->call<rst::geometry::Pose>("queryrobotpose");
    std::cout << "QueryPose returned rst::geometry::pose: Translation (x,y,z): " << pose->translation().x() << "," << pose->translation().y() << "," << pose->translation().z() << " ";
    std::cout << " Rotation (qx,qy,qz,qw): " << pose->rotation().qx() << "," << pose->rotation().qy() << "," << pose->rotation().qz() << "," << pose->rotation().qw() << std::endl;

    //std::cout << "Server replied: " << *future.get(10.0) << std::endl;
    // Note: timeout is in seconds.

    return EXIT_SUCCESS;
}
// mark-end::body
