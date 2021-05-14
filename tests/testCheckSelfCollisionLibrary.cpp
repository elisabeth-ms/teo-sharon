#include "gtest/gtest.h"

#include "../libraries/CheckSelfCollisionLibrary/CheckSelfCollisionLibrary.hpp"

#include <ColorDebug.h>

namespace roboticslab
{

    /**
 * @brief Tests \ref CheckSelfCollisionLibrary.
 */
    class CheckSelfCollisionTest : public testing::Test
    {
    public:
        KDL::Chain chain;
        KDL::JntArray qmin;
        KDL::JntArray qmax;
        std::vector<fcl::CollisionObject> collisionObjects;
        typedef std::shared_ptr<fcl::CollisionGeometry> CollisionGeometryPtr_t;

        virtual void SetUp()
        {
            printf("Setting up!\n");
        }

        virtual void TearDown()
        {
            delete checkSelfCollision;
            checkSelfCollision = 0;
        }

    protected:
        CheckSelfCollision *checkSelfCollision;

        static KDL::Chain makeTeoTrunkAndRightArmKinematicsFromDH()
        {
            const KDL::Joint rotZ(KDL::Joint::RotZ);
            KDL::Chain chain;
            chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0.0, -KDL::PI / 2, 0.1932, 0.0)));
            chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0.305, 0.0, -0.34692, -KDL::PI / 2)));
            chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, 0, 0)));
            chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, 0, -KDL::PI / 2)));
            chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, -0.32901, -KDL::PI / 2)));
            chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, KDL::PI / 2, 0, 0)));
            chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, -0.215, 0)));
            chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(-0.09, 0, 0, -KDL::PI / 2)));

            return chain;
        }
        void makeQLimitsTeoTrunkAndRightArmKinematicsFromDH()
        {
            qmin.resize(8);
            qmax.resize(8);
            qmin(0) = -59.3;
            qmin(1) = -15.4;
            qmin(2) = -98.1;
            qmin(3) = -75.5;
            qmin(4) = -80.1;
            qmin(5) = -99.6;
            qmin(6) = -80.4;
            qmin(7) = -115.1;
            qmax(0) = 46.3;
            qmax(1) = 10.1;
            qmax(2) = 106.0;
            qmax(3) = 22.4;
            qmax(4) = 57.0;
            qmax(5) = 98.4;
            qmax(6) = 99.6;
            qmax(7) = 44.7;
        }

        void makeTeoTrunkRightArmChainAndLimits()
        {
            chain = makeTeoTrunkAndRightArmKinematicsFromDH();
            makeQLimitsTeoTrunkAndRightArmKinematicsFromDH();
            fcl::Transform3f tfTest{fcl::Vec3f{0.0, 0.0, 0.0}};
            CollisionGeometryPtr_t collisionGeometry{new fcl::Box{0, 0, 0}};
            fcl::CollisionObject collisionObject(collisionGeometry, tfTest);
            int nOfCollisionObjects = 5;
            collisionObjects.reserve(nOfCollisionObjects);
            for(int i=0; i<nOfCollisionObjects; i++)
                collisionObjects.emplace_back(collisionObject);
        }
    };

    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionConstructorEmptyChain)
    {
        EXPECT_THROW(new CheckSelfCollision(chain, qmin, qmax, collisionObjects), std::runtime_error) << "Chain is empty but no exception thrown";
    }
    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionConstructorEmptyQmin){
        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.,0.,0.1,0.0)));
        ASSERT_EQ(chain.getNrOfJoints(),1);
        EXPECT_THROW(new CheckSelfCollision(chain, qmin, qmax, collisionObjects), std::runtime_error)<<"qmin is empty but no exception thrown";
    }

    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionConstructorEmptyQmax){
        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.,0.,0.1,0.0)));
        qmin.resize(chain.getNrOfJoints());
        EXPECT_THROW(new CheckSelfCollision(chain, qmin, qmax, collisionObjects), std::runtime_error)<<"qmax is empty but no exception thrown";
    }
    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionConstructorEqualNJointsQLimits){
        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.,0.,0.1,0.0)));
        qmin.resize(2);
        qmax.resize(1);
        EXPECT_THROW(new CheckSelfCollision(chain, qmin, qmax, collisionObjects), std::runtime_error)<<"Chain number of joints is not equal to qlimits size, but no exception is thrown";
    }

    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionConstructorTeoTrunkRightArmFromDH)
    {
        //ASSERT_TRUE(exampleLibrary->expectTrueResult());

        makeTeoTrunkRightArmChainAndLimits();
        checkSelfCollision = new CheckSelfCollision(chain, qmin, qmax, collisionObjects);

        ASSERT_EQ(checkSelfCollision->chain.getNrOfJoints(), 8);
        ASSERT_EQ(checkSelfCollision->chain.getNrOfSegments(), 8);
        for (int i=0; i<chain.getNrOfJoints(); i++){
            ASSERT_EQ(checkSelfCollision->qmin(i), qmin(i));
            ASSERT_EQ(checkSelfCollision->qmax(i), qmax(i));
        }
    }

    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionJointsOutsideBoundsException){
        makeTeoTrunkRightArmChainAndLimits();

        KDL::JntArray q(chain.getNrOfJoints());
        for(int i=0; i<qmin.rows(); i++){
            q(i) = qmin(i);
            printf("%f %f %f\n",q, qmin(0), qmax(0));
        }
        q(0) = 500;

        checkSelfCollision = new CheckSelfCollision(chain, qmin, qmax, collisionObjects);

        ASSERT_EQ(checkSelfCollision->jointsInsideBounds(q), false)<<"joint is outside bounds but returns true";
    }

    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionCenterLinks){
        makeTeoTrunkRightArmChainAndLimits();
        checkSelfCollision = new CheckSelfCollision(chain, qmin, qmax, collisionObjects);

        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(0).second.p.x(), 0, 0.0001);
        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(0).second.p.y(), 0, 0.0001);
        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(0).second.p.z(), 0.1932/2.0, 0.0001);

        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(1).second.p.x(), 0, 0.0001);
        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(1).second.p.y(), -0.305/2.0, 0.0001);
        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(1).second.p.z(), -0.34692/2.0, 0.0001);

        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(2).second.p.x(), 0, 0.0001);
        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(2).second.p.y(), 0, 0.0001);
        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(2).second.p.z(), -0.32910/2.0, 0.0001);

        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(3).second.p.x(), 0, 0.0001);
        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(3).second.p.y(), 0, 0.0001);
        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(3).second.p.z(), -0.2150/2.0, 0.0001);

        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(4).second.p.x(), 0, 0.0001);
        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(4).second.p.y(), 0.09/2.0, 0.0001);
        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(4).second.p.z(), 0.0, 0.0001);

    }

    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionFKJntToCart){
        makeTeoTrunkRightArmChainAndLimits();
        int kinematics_status;
        KDL::ChainFkSolverPos_recursive fksolver(chain);
        KDL::JntArray jointPositions(chain.getNrOfJoints());
        KDL::Frame frameJoint;

        kinematics_status = fksolver.JntToCart(jointPositions, frameJoint, 1);
        printf("%f %f %f\n",frameJoint.p.x(), frameJoint.p.y(), frameJoint.p.z());
        ASSERT_FALSE(kinematics_status);

    }

    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionNofCollsionObjectsError){
        makeTeoTrunkRightArmChainAndLimits();
        collisionObjects.pop_back();
        ASSERT_THROW( new CheckSelfCollision(chain, qmin, qmax, collisionObjects), std::runtime_error);
    }

    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionTwoLinksCollide){
        makeTeoTrunkRightArmChainAndLimits();
        collisionObjects.clear();
        CollisionGeometryPtr_t teoRootTrunk{new fcl::Box{0.35, 0.35, 0.1932}};
        fcl::Transform3f tfTest{fcl::Vec3f{0.0, 0.0, 0.0}};
        fcl::CollisionObject collisionObject1{teoRootTrunk,tfTest};

        CollisionGeometryPtr_t teoTrunk{new fcl::Box{0.1, 0.35, 0.1}};
        fcl::CollisionObject collisionObject2{teoTrunk,tfTest};
        int nOfCollisionObjects = 5;
        collisionObjects.reserve(nOfCollisionObjects);
        collisionObjects.emplace_back(collisionObject1);
        collisionObjects.emplace_back(collisionObject2);

        for(int i=0; i<nOfCollisionObjects-2; i++)
            collisionObjects.emplace_back(collisionObject1);

        checkSelfCollision = new CheckSelfCollision(chain, qmin, qmax, collisionObjects);
        KDL::JntArray q(8);
        q(0) = 10;
        q(1) = -8;
        checkSelfCollision->updateCollisionObjectsTransform(q);

        ASSERT_TRUE(checkSelfCollision->twoLinksCollide(q,0,1));
    }

} // namespace roboticslab
