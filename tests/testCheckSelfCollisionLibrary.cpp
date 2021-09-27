#include "gtest/gtest.h"

#include "../libraries/CheckSelfCollisionLibrary/CheckSelfCollisionLibrary.hpp"

#include <ColorDebug.h>

namespace sharon
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
        std::vector<fcl::CollisionObjectf> collisionObjects;
        std::vector<std::array<float, 3>> offsetCollisionObjects;
        std::vector<fcl::CollisionObjectf> tableCollision;
        typedef std::shared_ptr<fcl::CollisionGeometryf> CollisionGeometryPtr_t;

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
            const KDL::Joint fixed(KDL::Joint::None); //Rigid Connection

            KDL::Chain chain;
            chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0.0, -KDL::PI / 2, 0.1932, 0.0)));
            chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0.305, 0.0, -0.34692, -KDL::PI / 2)));
            chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, 0, 0)));
            chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, 0, -KDL::PI / 2)));
            chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, -0.32901, -KDL::PI / 2)));
            chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, KDL::PI / 2, 0, 0)));
            chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, -0.215, 0)));
            chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(-0.09, 0, 0, -KDL::PI / 2)));
            chain.addSegment(KDL::Segment(fixed, KDL::Frame::DH(0, KDL::PI / 2, 0, -KDL::PI / 2)));
            chain.addSegment(KDL::Segment(fixed, KDL::Frame::DH(0, 0, 0.0975, 0)));

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
            fcl::Transform3f tfTest;
            CollisionGeometryPtr_t collisionGeometry{new fcl::Boxf{0, 0, 0}};
            fcl::CollisionObjectf collisionObject(collisionGeometry, tfTest);
            int nOfCollisionObjects = 6;
            collisionObjects.reserve(nOfCollisionObjects);
            offsetCollisionObjects.reserve(nOfCollisionObjects);
            std::array<float, 3> offsetObject = {0, 0, 0};
            for (int i = 0; i < nOfCollisionObjects; i++)
            {
                collisionObjects.emplace_back(collisionObject);
                offsetCollisionObjects.emplace_back(offsetObject);
            }
        }
    };

    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionConstructorEmptyChain)
    {
        EXPECT_THROW(new CheckSelfCollision(chain, qmin, qmax, collisionObjects, offsetCollisionObjects,tableCollision), std::runtime_error) << "Chain is empty but no exception thrown";
    }
    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionConstructorEmptyQmin)
    {
        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0., 0., 0.1, 0.0)));
        ASSERT_EQ(chain.getNrOfJoints(), 1);
        EXPECT_THROW(new CheckSelfCollision(chain, qmin, qmax, collisionObjects, offsetCollisionObjects, tableCollision), std::runtime_error) << "qmin is empty but no exception thrown";
    }

    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionConstructorEmptyQmax)
    {
        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0., 0., 0.1, 0.0)));
        qmin.resize(chain.getNrOfJoints());
        EXPECT_THROW(new CheckSelfCollision(chain, qmin, qmax, collisionObjects, offsetCollisionObjects, tableCollision), std::runtime_error) << "qmax is empty but no exception thrown";
    }
    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionConstructorEqualNJointsQLimits)
    {
        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0., 0., 0.1, 0.0)));
        qmin.resize(2);
        qmax.resize(1);
        EXPECT_THROW(new CheckSelfCollision(chain, qmin, qmax, collisionObjects, offsetCollisionObjects, tableCollision), std::runtime_error) << "Chain number of joints is not equal to qlimits size, but no exception is thrown";
    }
    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionConstructorEqualNOffsetObjects)
    {
        makeTeoTrunkRightArmChainAndLimits();
        offsetCollisionObjects.pop_back();
        EXPECT_THROW(new CheckSelfCollision(chain, qmin, qmax, collisionObjects, offsetCollisionObjects, tableCollision), std::runtime_error) << "Size of offsetCenterCollisionObjects is not equal to size collisionObjects, but no exception is thrown";
    }

    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionConstructorTeoTrunkRightArmFromDH)
    {
        //ASSERT_TRUE(exampleLibrary->expectTrueResult());

        makeTeoTrunkRightArmChainAndLimits();
        checkSelfCollision = new CheckSelfCollision(chain, qmin, qmax, collisionObjects, offsetCollisionObjects, tableCollision);

        ASSERT_EQ(checkSelfCollision->chain.getNrOfJoints(), 8);
        ASSERT_EQ(checkSelfCollision->chain.getNrOfSegments(), 10);
        for (int i = 0; i < chain.getNrOfJoints(); i++)
        {
            ASSERT_EQ(checkSelfCollision->qmin(i), qmin(i));
            ASSERT_EQ(checkSelfCollision->qmax(i), qmax(i));
        }
    }

    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionJointsOutsideBoundsException)
    {
        makeTeoTrunkRightArmChainAndLimits();

        KDL::JntArray q(chain.getNrOfJoints());
        for (int i = 0; i < qmin.rows(); i++)
        {
            q(i) = qmin(i);
            printf("%f %f %f\n", q, qmin(0), qmax(0));
        }
        q(0) = 500;

        checkSelfCollision = new CheckSelfCollision(chain, qmin, qmax, collisionObjects, offsetCollisionObjects, tableCollision);

        ASSERT_EQ(checkSelfCollision->jointsInsideBounds(q), false) << "joint is outside bounds but returns true";
    }

    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionCenterLinks)
    {
        makeTeoTrunkRightArmChainAndLimits();
        checkSelfCollision = new CheckSelfCollision(chain, qmin, qmax, collisionObjects, offsetCollisionObjects, tableCollision);

        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(0).second.p.x(), 0, 0.0001);
        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(0).second.p.y(), 0, 0.0001);
        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(0).second.p.z(), 0.1932 / 2.0, 0.0001);

        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(1).second.p.x(), 0, 0.0001);
        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(1).second.p.y(), -0.305 / 2.0, 0.0001);
        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(1).second.p.z(), -0.34692 / 2.0, 0.0001);

        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(2).second.p.x(), 0, 0.0001);
        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(2).second.p.y(), 0, 0.0001);
        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(2).second.p.z(), -0.32910 / 2.0, 0.0001);

        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(3).second.p.x(), 0, 0.0001);
        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(3).second.p.y(), 0, 0.0001);
        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(3).second.p.z(), -0.2150 / 2.0, 0.0001);

        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(4).second.p.x(), 0, 0.0001);
        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(4).second.p.y(), 0.09 / 2.0, 0.0001);
        ASSERT_NEAR(checkSelfCollision->centerLinksWrtJoints.at(4).second.p.z(), 0.0, 0.0001);
    }

    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionFKJntToCart)
    {
        makeTeoTrunkRightArmChainAndLimits();
        int kinematics_status;
        KDL::ChainFkSolverPos_recursive fksolver(chain);
        KDL::JntArray jointPositions(chain.getNrOfJoints());
        KDL::Frame frameJoint;

        kinematics_status = fksolver.JntToCart(jointPositions, frameJoint, 1);
        printf("%f %f %f\n", frameJoint.p.x(), frameJoint.p.y(), frameJoint.p.z());
        ASSERT_FALSE(kinematics_status);
    }

    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionNofCollisionObjectsError)
    {
        makeTeoTrunkRightArmChainAndLimits();
        collisionObjects.pop_back();
        ASSERT_THROW(new CheckSelfCollision(chain, qmin, qmax, collisionObjects, offsetCollisionObjects, tableCollision), std::runtime_error);
    }



    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionTwoLinksCollide)
    {
        makeTeoTrunkRightArmChainAndLimits();
        CollisionGeometryPtr_t teoRootTrunk{new fcl::Boxf{0.3, 0.25, 2.0}};
        fcl::Transform3f tfTest;
        fcl::CollisionObjectf collisionObject1{teoRootTrunk, tfTest};

        CollisionGeometryPtr_t teoTrunk{new fcl::Boxf{0.3, 0.3, 0.46}};
        fcl::CollisionObjectf collisionObject2{teoTrunk, tfTest};

        CollisionGeometryPtr_t teoAxialShoulder{new fcl::Boxf{0.10,0.10,0.32901}};//{new fcl::Box{0.15, 0.15, 0.32901}};
        fcl::CollisionObjectf collisionObject3{teoAxialShoulder, tfTest};

        CollisionGeometryPtr_t teoElbow{new fcl::Boxf{0.10, 0.10, 0.22}};
        fcl::CollisionObjectf collisionObject4{teoElbow, tfTest};

        CollisionGeometryPtr_t teoWrist{new fcl::Boxf{0.2, 0.20, 0.2}};
        fcl::CollisionObjectf collisionObject5{teoWrist, tfTest};


        CollisionGeometryPtr_t teoEndEffector{new fcl::Boxf{0.0,0.0,0.0}};
        fcl::CollisionObjectf collisionObject6{teoEndEffector, tfTest};

        int nOfCollisionObjects = 6;
        collisionObjects.clear();
        collisionObjects.reserve(nOfCollisionObjects);
        collisionObjects.emplace_back(collisionObject1);
        collisionObjects.emplace_back(collisionObject2);
        collisionObjects.emplace_back(collisionObject3);
        collisionObjects.emplace_back(collisionObject4);
        collisionObjects.emplace_back(collisionObject5);
        collisionObjects.emplace_back(collisionObject6);

        offsetCollisionObjects[0][2] = -0.2;

        offsetCollisionObjects[1][1] = 0.0;
        offsetCollisionObjects[1][2] = +0.1734;

        offsetCollisionObjects[4][1] = 0.055;


        checkSelfCollision = new CheckSelfCollision(chain, qmin, qmax, collisionObjects, offsetCollisionObjects, tableCollision);
        KDL::JntArray q(8);
        q(0) = 0;
        q(1) = 0;
        q(2) = 0;
        checkSelfCollision->updateCollisionObjectsTransform(q);
        ASSERT_TRUE(checkSelfCollision->twoLinksCollide(q, 0, 1));
        ASSERT_FALSE(checkSelfCollision->twoLinksCollide(q, 0, 2));
        ASSERT_FALSE(checkSelfCollision->twoLinksCollide(q, 0, 3));
        ASSERT_FALSE(checkSelfCollision->twoLinksCollide(q, 0, 4));
        ASSERT_FALSE(checkSelfCollision->twoLinksCollide(q, 0, 5));
        

       
        q(0) = -0.0367917881081746347971;
        q(1) = 14.2365087575865114644;
        q(2) = 27.8665735645256624764;
        q(3) = -10.6261833845321103098;
        q(4) = 14.4323992893486110489;
        q(5) = -58.9686125917134518204;
        q(6) = 39.974431579071165288;
        q(7) = -19.9670119544058799477;

        checkSelfCollision->updateCollisionObjectsTransform(q);
        ASSERT_TRUE(checkSelfCollision->twoLinksCollide(q, 0, 1));
        ASSERT_FALSE(checkSelfCollision->twoLinksCollide(q, 0, 2));
        ASSERT_FALSE(checkSelfCollision->twoLinksCollide(q, 0, 3));
        ASSERT_FALSE(checkSelfCollision->twoLinksCollide(q, 0, 4));
        ASSERT_FALSE(checkSelfCollision->twoLinksCollide(q, 0, 5));



        q(0) = 0;
        q(1) = 0;
        q(2) = 0;
        q(3) = 15;
        q(4) = 0;
        q(5) = 0.0;
        q(6) = 0.0;
        q(7) = 0.0;
        checkSelfCollision->updateCollisionObjectsTransform(q);
        ASSERT_TRUE(checkSelfCollision->twoLinksCollide(q, 1, 2));

        q(0) = 0;
        q(1) = 0;
        q(2) = -30;
        q(3) = 5;
        checkSelfCollision->updateCollisionObjectsTransform(q);
        ASSERT_FALSE(checkSelfCollision->twoLinksCollide(q, 1, 2));

        q(0) = 0;
        q(1) = 0;
        q(2) = -50;
        q(3) = 10;
        checkSelfCollision->updateCollisionObjectsTransform(q);
        ASSERT_FALSE(checkSelfCollision->twoLinksCollide(q, 1, 2));

        q(0) = 10;
        q(1) = -10;
        q(2) = -50;
        q(3) = 12;
        checkSelfCollision->updateCollisionObjectsTransform(q);
        ASSERT_FALSE(checkSelfCollision->twoLinksCollide(q, 1, 2));

        q(0) = 10;
        q(1) = 10;
        q(2) = -10;
        q(3) = 15;
        checkSelfCollision->updateCollisionObjectsTransform(q);
        ASSERT_TRUE(checkSelfCollision->twoLinksCollide(q, 1, 2));

        q(0) = 10;
        q(1) = -10;
        q(2) = -10;
        q(3) = -15;
        q(4) = 10;
        q(5) = -10;

        for (int val = qmin(3); val < qmax(3); val++)
        {
            q(3) = val;
            checkSelfCollision->updateCollisionObjectsTransform(q);
            ASSERT_TRUE(checkSelfCollision->twoLinksCollide(q, 2, 3));
        }


        q(0) = 0;
        q(1) = 0;
        q(2) = -10;
        q(3) = 10;
        q(4) = 60;
        q(5) = -60;

        checkSelfCollision->updateCollisionObjectsTransform(q);
        ASSERT_TRUE(checkSelfCollision->twoLinksCollide(q, 0, 3));

        q(0) = 0;
        q(1) = 0;
        q(2) = -18;
        q(3) = -10;
        q(4) = 60;
        q(5) = -60;

        checkSelfCollision->updateCollisionObjectsTransform(q);
        ASSERT_FALSE(checkSelfCollision->twoLinksCollide(q, 0, 3));
        ASSERT_FALSE(checkSelfCollision->twoLinksCollide(q, 0, 4));

        checkSelfCollision->updateCollisionObjectsTransform(q);
        ASSERT_TRUE(checkSelfCollision->twoLinksCollide(q, 3, 4));


        q(0) = 0;
        q(1) = 0;
        q(2) = -10;
        q(3) = -10;
        q(4) = 50;
        q(5) = -80;
        q(6) = -10;
        q(7) = -90;

        checkSelfCollision->updateCollisionObjectsTransform(q);
        ASSERT_TRUE(checkSelfCollision->twoLinksCollide(q, 1, 4));

        q(0) = 0;
        q(1) = 0;
        q(2) = -10;
        q(3) = -30;
        q(4) = 50;
        q(5) = -80;
        q(6) = -10;
        q(7) = -90;

        checkSelfCollision->updateCollisionObjectsTransform(q);
        ASSERT_FALSE(checkSelfCollision->twoLinksCollide(q, 1, 4));

        q(0) = 0;
        q(1) = 0;
        q(2) = 0;
        q(3) = 0;
        q(4) = 80;
        q(5) = -20;
        q(6) = 0;
        q(7) = 0;

        checkSelfCollision->updateCollisionObjectsTransform(q);
        ASSERT_TRUE(checkSelfCollision->twoLinksCollide(q, 0, 4));

    }

/**

 TEST_F(CheckSelfCollisionTest, CheckSelfCollision)
    {
        makeTeoTrunkRightArmChainAndLimits();
        CollisionGeometryPtr_t teoRootTrunk{new fcl::Boxf{0.25, 0.25, 0.6}};
        fcl::Transform3f tfTest;
        fcl::CollisionObjectf collisionObject1{teoRootTrunk, tfTest};

        CollisionGeometryPtr_t teoTrunk{new fcl::Boxf{0.3, 0.3, 0.46}};
        fcl::CollisionObjectf collisionObject2{teoTrunk, tfTest};

        CollisionGeometryPtr_t teoAxialShoulder{new fcl::Boxf{0.10,0.10,0.32901}};//{new fcl::Box{0.15, 0.15, 0.32901}};
        fcl::CollisionObjectf collisionObject3{teoAxialShoulder, tfTest};

        CollisionGeometryPtr_t teoElbow{new fcl::Boxf{0.10, 0.10, 0.22}};
        fcl::CollisionObjectf collisionObject4{teoElbow, tfTest};

        CollisionGeometryPtr_t teoWrist{new fcl::Boxf{0.2, 0.20, 0.2}};
        fcl::CollisionObjectf collisionObject5{teoWrist, tfTest};

        int nOfCollisionObjects = 5;
        collisionObjects.clear();
        collisionObjects.reserve(nOfCollisionObjects);
        collisionObjects.emplace_back(collisionObject1);
        collisionObjects.emplace_back(collisionObject2);
        collisionObjects.emplace_back(collisionObject3);
        collisionObjects.emplace_back(collisionObject4);
        collisionObjects.emplace_back(collisionObject5);

        offsetCollisionObjects[0][2] = -0.2;

        offsetCollisionObjects[1][1] = 0.0;
        offsetCollisionObjects[1][2] = +0.1734;

        offsetCollisionObjects[4][1] = 0.055;

        CollisionGeometryPtr_t table{new fcl::Boxf{0.8,1.5,0.9}};
        fcl::CollisionObjectf collisionObjectTable{table, tfTest};
        fcl::Vector3f translation(0.6, -0.0, -0.45);
        collisionObjectTable.setTranslation(translation);
        tableCollision.clear();
        tableCollision.push_back(collisionObjectTable);


        checkSelfCollision = new CheckSelfCollision(chain, qmin, qmax, collisionObjects, offsetCollisionObjects, tableCollision);
        KDL::JntArray q(8);
        q(0) = 0;
        q(1) = -8;
        checkSelfCollision->updateCollisionObjectsTransform(q);
        ASSERT_FALSE(checkSelfCollision->selfCollision());

        q(0) = 0;
        q(1) = 0;
        q(2) = 0;
        q(3) = 0;
        q(4) = 80;
        q(5) = -20;
        q(6) = 0;
        q(7) = 0;

        checkSelfCollision->updateCollisionObjectsTransform(q);
        ASSERT_TRUE(checkSelfCollision->selfCollision());


    }

TEST_F(CheckSelfCollisionTest, CheckSelfCollisionTwoLinksDistance)
    {
        makeTeoTrunkRightArmChainAndLimits();
        CollisionGeometryPtr_t teoRootTrunk{new fcl::Boxf{0.25, 0.25, 0.6}};
        fcl::Transform3f tfTest;
        fcl::CollisionObjectf collisionObject1{teoRootTrunk, tfTest};

        CollisionGeometryPtr_t teoTrunk{new fcl::Boxf{0.3, 0.3, 0.46}};
        fcl::CollisionObjectf collisionObject2{teoTrunk, tfTest};

        CollisionGeometryPtr_t teoAxialShoulder{new fcl::Boxf{0.10,0.10,0.32901}};//{new fcl::Box{0.15, 0.15, 0.32901}};
        fcl::CollisionObjectf collisionObject3{teoAxialShoulder, tfTest};

        CollisionGeometryPtr_t teoElbow{new fcl::Boxf{0.10, 0.10, 0.22}};
        fcl::CollisionObjectf collisionObject4{teoElbow, tfTest};

        CollisionGeometryPtr_t teoWrist{new fcl::Boxf{0.2, 0.20, 0.2}};
        fcl::CollisionObjectf collisionObject5{teoWrist, tfTest};

        int nOfCollisionObjects = 5;
        collisionObjects.clear();
        collisionObjects.reserve(nOfCollisionObjects);
        collisionObjects.emplace_back(collisionObject1);
        collisionObjects.emplace_back(collisionObject2);
        collisionObjects.emplace_back(collisionObject3);
        collisionObjects.emplace_back(collisionObject4);
        collisionObjects.emplace_back(collisionObject5);

        offsetCollisionObjects[0][2] = -0.2;

        offsetCollisionObjects[1][1] = 0.0;
        offsetCollisionObjects[1][2] = +0.1734;

        offsetCollisionObjects[4][1] = 0.055;




        checkSelfCollision = new CheckSelfCollision(chain, qmin, qmax, collisionObjects, offsetCollisionObjects, tableCollision);
        KDL::JntArray q(8);
        q(0) = 0;
        q(1) = 0;
        q(2) = -10;
        q(3) = -30;
        q(4) = 50;
        q(5) = -80;
        q(6) = -10;
        q(7) = -90;
        checkSelfCollision->updateCollisionObjectsTransform(q);
        ASSERT_NEAR(checkSelfCollision->twoLinksDistance(q, 1, 4),0.0381804,0.0001);

        q(0) = 0;
        q(1) = 0;
        q(2) = 0;
        q(3) = 40;
        q(4) = 0;
        q(5) = 0;
        q(6) = 0;
        q(7) = 0;

        checkSelfCollision->updateCollisionObjectsTransform(q);
        ASSERT_TRUE(checkSelfCollision->twoLinksCollide(q, 0, 3));
        ASSERT_NEAR(checkSelfCollision->twoLinksDistance(q, 0, 3),-0.14919,0.0001);

    }

    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionminDistance)
    {
        makeTeoTrunkRightArmChainAndLimits();
        CollisionGeometryPtr_t teoRootTrunk{new fcl::Boxf{0.25, 0.25, 0.6}};
        fcl::Transform3f tfTest;
        fcl::CollisionObjectf collisionObject1{teoRootTrunk, tfTest};

        CollisionGeometryPtr_t teoTrunk{new fcl::Boxf{0.3, 0.3, 0.46}};
        fcl::CollisionObjectf collisionObject2{teoTrunk, tfTest};

        CollisionGeometryPtr_t teoAxialShoulder{new fcl::Boxf{0.10,0.10,0.32901}};//{new fcl::Box{0.15, 0.15, 0.32901}};
        fcl::CollisionObjectf collisionObject3{teoAxialShoulder, tfTest};

        CollisionGeometryPtr_t teoElbow{new fcl::Boxf{0.10, 0.10, 0.22}};
        fcl::CollisionObjectf collisionObject4{teoElbow, tfTest};

        CollisionGeometryPtr_t teoWrist{new fcl::Boxf{0.2, 0.20, 0.2}};
        fcl::CollisionObjectf collisionObject5{teoWrist, tfTest};

        int nOfCollisionObjects = 5;
        collisionObjects.clear();
        collisionObjects.reserve(nOfCollisionObjects);
        collisionObjects.emplace_back(collisionObject1);
        collisionObjects.emplace_back(collisionObject2);
        collisionObjects.emplace_back(collisionObject3);
        collisionObjects.emplace_back(collisionObject4);
        collisionObjects.emplace_back(collisionObject5);

        offsetCollisionObjects[0][2] = -0.2;

        offsetCollisionObjects[1][1] = 0.0;
        offsetCollisionObjects[1][2] = +0.1734;

        offsetCollisionObjects[4][1] = 0.055;




        checkSelfCollision = new CheckSelfCollision(chain, qmin, qmax, collisionObjects, offsetCollisionObjects, tableCollision);
        KDL::JntArray q(8);
        q(0) = 0;
        q(1) = 0;
        q(2) = -10;
        q(3) = -30;
        q(4) = 50;
        q(5) = -80;
        q(6) = -10;
        q(7) = -90;
        checkSelfCollision->updateCollisionObjectsTransform(q);
        ASSERT_NEAR(checkSelfCollision->minDistance(),0.0381804,0.0001);

        q(0) = 0;
        q(1) = 0;
        q(2) = 0;
        q(3) = 40;
        q(4) = 0;
        q(5) = 0;
        q(6) = 0;
        q(7) = 0;

        checkSelfCollision->updateCollisionObjectsTransform(q);
        ASSERT_TRUE(checkSelfCollision->twoLinksCollide(q, 0, 3));
        ASSERT_NEAR(checkSelfCollision->minDistance(),-0.14919,0.0001);



    }
**/
    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionTable)
    {
        makeTeoTrunkRightArmChainAndLimits();
        CollisionGeometryPtr_t teoRootTrunk{new fcl::Boxf{0.25, 0.25, 0.6}};
        fcl::Transform3f tfTest;
        fcl::CollisionObjectf collisionObject1{teoRootTrunk, tfTest};

        CollisionGeometryPtr_t teoTrunk{new fcl::Boxf{0.3, 0.3, 0.46}};
        fcl::CollisionObjectf collisionObject2{teoTrunk, tfTest};
        

        CollisionGeometryPtr_t teoAxialShoulder{new fcl::Boxf{0.10,0.10,0.32901}};//{new fcl::Box{0.15, 0.15, 0.32901}};
        fcl::CollisionObjectf collisionObject3{teoAxialShoulder, tfTest};

        CollisionGeometryPtr_t teoElbow{new fcl::Boxf{0.10, 0.10, 0.22}};
        fcl::CollisionObjectf collisionObject4{teoElbow, tfTest};

        CollisionGeometryPtr_t teoWrist{new fcl::Boxf{0.2, 0.20, 0.2}};
        fcl::CollisionObjectf collisionObject5{teoWrist, tfTest};

        CollisionGeometryPtr_t teoEndEffector{new fcl::Boxf{0.1,0.1,0.23}};
        fcl::CollisionObjectf collisionObject6{teoEndEffector, tfTest};



        int nOfCollisionObjects = 6;
        collisionObjects.clear();
        collisionObjects.reserve(nOfCollisionObjects);
        collisionObjects.emplace_back(collisionObject1);
        collisionObjects.emplace_back(collisionObject2);
        collisionObjects.emplace_back(collisionObject3);
        collisionObjects.emplace_back(collisionObject4);
        collisionObjects.emplace_back(collisionObject5);
        collisionObjects.emplace_back(collisionObject6);

        offsetCollisionObjects[0][2] = -0.2;

        offsetCollisionObjects[1][1] = 0.0;
        offsetCollisionObjects[1][2] = +0.1734;

        offsetCollisionObjects[4][1] = 0.055;

        CollisionGeometryPtr_t table{new fcl::Boxf{1.0,1.0,0.9}};
        fcl::CollisionObjectf collisionObjectTable{table, tfTest};
        fcl::Quaternionf rotation(1.0,0.0,0.0,0.0);
        fcl::Vector3f translation(0.8, 0.0, -0.45);
        collisionObjectTable.setTransform(rotation, translation);
        tableCollision.clear();
        tableCollision.reserve(1);
        tableCollision.emplace_back(collisionObjectTable);


        checkSelfCollision = new CheckSelfCollision(chain, qmin, qmax, collisionObjects, offsetCollisionObjects, tableCollision);
        KDL::JntArray q(8);
        q(0) = 0;
        q(1) = 40;
        q(2) = -40;
        checkSelfCollision->updateCollisionObjectsTransform(q);
        ASSERT_FALSE(checkSelfCollision->linkTableCollide(q,1));
        ASSERT_FALSE(checkSelfCollision->linkTableCollide(q,3));

        // q(2) = -30;
        // q(5) = -20;
        // checkSelfCollision->updateCollisionObjectsTransform(q);
        // ASSERT_TRUE(checkSelfCollision->linkTableCollide(q,3));

        // q(2) = -30;
        // q(5) = -10;
        // checkSelfCollision->updateCollisionObjectsTransform(q);
        // ASSERT_FALSE(checkSelfCollision->linkTableCollide(q,3));
        // ASSERT_TRUE(checkSelfCollision->linkTableCollide(q,4));



        q(0) = 0;
        q(1) = 15.15;
        q(2) = 10.63;
        q(3) = 0;
        q(4) = 0;
        q(5) = -70.04;
        q(6) = 80.003;
        q(7) = 0.93;
        checkSelfCollision->updateCollisionObjectsTransform(q);

        
        ASSERT_FALSE(checkSelfCollision->linkTableCollide(q,0));
        ASSERT_FALSE(checkSelfCollision->linkTableCollide(q,1));
        ASSERT_FALSE(checkSelfCollision->linkTableCollide(q,2));
        ASSERT_FALSE(checkSelfCollision->linkTableCollide(q,3));
        ASSERT_TRUE(checkSelfCollision->linkTableCollide(q,4));


        // ASSERT_TRUE(checkSelfCollision->selfCollision());


    }
    
} // namespace sharon
