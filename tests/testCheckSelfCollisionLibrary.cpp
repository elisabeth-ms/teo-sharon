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

        static KDL::Chain makeTeoFixedTrunkAndRightArmKinematicsFromDH()
        {
            const KDL::Joint rotZ(KDL::Joint::RotZ);
            const KDL::Joint fixed(KDL::Joint::None); //Rigid Connection
            KDL::Chain chain;
            chain.addSegment(KDL::Segment(fixed, KDL::Frame::DH(0.0, -KDL::PI / 2, 0.1932, 0.0)));
            chain.addSegment(KDL::Segment(fixed, KDL::Frame::DH(0.305, 0.0, -0.34692, -KDL::PI / 2)));
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
        void makeQLimitsTeoFixedTrunkAndRightArmKinematicsFromDH(){
            qmin(0) = -98.1; qmin(1) = -75.5; qmin(2) = -80.1; qmin(3) = -99.6; qmin(4) = -80.4; qmin(5) = -115.1;
            qmax(0) = 106.0; qmax(1) = 22.4;  qmax(2) = 57.0;  qmax(3) = 98.4;  qmax(4) = 99.6;  qmax(5) = 44.7;
        }
    };

    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionConstructorEmptyChain){
        EXPECT_THROW(new CheckSelfCollision(chain, qmin, qmax), std::runtime_error)<<"Chain is empty but no exception thrown";
    }
    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionConstructorEmptyQmin){
        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.,0.,0.1,0.0)));
        ASSERT_EQ(chain.getNrOfJoints(),1);
        EXPECT_THROW(new CheckSelfCollision(chain, qmin, qmax), std::runtime_error)<<"qmin is empty but no exception thrown";
    }

    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionConstructorEmptyQmax){
        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.,0.,0.1,0.0)));
        qmin.resize(chain.getNrOfJoints());
        EXPECT_THROW(new CheckSelfCollision(chain, qmin, qmax), std::runtime_error)<<"qmax is empty but no exception thrown";
    }
    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionConstructorEqualNJointsQLimits){
        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.,0.,0.1,0.0)));
        qmin.resize(2);
        qmax.resize(1);
        EXPECT_THROW(new CheckSelfCollision(chain, qmin, qmax), std::runtime_error)<<"Chain number of joints is not equal to qlimits size, but no exception is thrown";
    }



    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionConstructorTeoFixedTrunkRightArmFromDH)
    {
        //ASSERT_TRUE(exampleLibrary->expectTrueResult());

        chain = makeTeoFixedTrunkAndRightArmKinematicsFromDH();
        qmin.resize(chain.getNrOfJoints());
        qmax.resize(chain.getNrOfJoints());
        makeQLimitsTeoFixedTrunkAndRightArmKinematicsFromDH();
        checkSelfCollision = new CheckSelfCollision(chain, qmin, qmax);

        ASSERT_EQ(checkSelfCollision->chain.getNrOfJoints(), 6); 
        ASSERT_EQ(checkSelfCollision->chain.getNrOfSegments(), 10);
        for (int i=0; i<chain.getNrOfJoints(); i++){
            ASSERT_EQ(checkSelfCollision->qmin(i), qmin(i));
            ASSERT_EQ(checkSelfCollision->qmax(i), qmax(i));
        }
    }

    TEST_F(CheckSelfCollisionTest, CheckSelfCollisionJointsOutsideBoundsException){
        chain = makeTeoFixedTrunkAndRightArmKinematicsFromDH();
        qmin.resize(chain.getNrOfJoints());
        qmax.resize(chain.getNrOfJoints());
        makeQLimitsTeoFixedTrunkAndRightArmKinematicsFromDH();

        KDL::JntArray q(chain.getNrOfJoints());
        for(int i=0; i<qmin.rows(); i++){
            q(i) = qmin(i);
            printf("%f %f %f\n",q, qmin(0), qmax(0));
        }
        q(0) = 100;

        checkSelfCollision = new CheckSelfCollision(chain, qmin, qmax);

        ASSERT_EQ(checkSelfCollision->jointsInsideBounds(q), false)<<"joint is outside bounds but returns true";
    }

} // namespace roboticslab
