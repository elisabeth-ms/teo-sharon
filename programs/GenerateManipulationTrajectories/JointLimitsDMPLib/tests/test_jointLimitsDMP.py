from JointLimitsDMPLib import jointLimitsDMP
import numpy as np
import math
import pytest
def test_jointLimitsDMP():
    dt = 0.001
    execution_time = 1.0
    n_features = 10
    qmin = [-40,-60]
    qmax = [25,65]
    dmp_name = "2_joints"
    jointLimitsDMP1 = jointLimitsDMP.JointLimitsDMP(dt, execution_time, n_features, qmin, qmax, dmp_name, len(qmin))
    assert jointLimitsDMP1.dt == dt
    assert jointLimitsDMP1.execution_time == execution_time
    assert jointLimitsDMP1.n_features == n_features
    assert jointLimitsDMP1.q_min.size != 0
    assert jointLimitsDMP1.q_max.size != 0


def test_SameDimensions():
    dt = 0.001
    execution_time = 1.0
    n_features = 10
    qmin = [-40,-60]
    qmax = [25,65]
    dmp_name = "2_joints"
    jointLimitsDMP1 = jointLimitsDMP.JointLimitsDMP(dt, execution_time, n_features, qmin, qmax, dmp_name, len(qmin))
    y_demo = np.array([])
    x0 = np.zeros(len(qmin)+1)
    g = np.zeros(len(qmin)+1)
    with pytest.raises(Exception) as exception_info:
        correct, trajectory = jointLimitsDMP1.imitateDemoInsideBounds(y_demo, x0, g)
        assert str(exception_info.value) == 'demo.rows != x0.rows'
    
    g = np.zeros(2)
    with pytest.raises(Exception) as exception_info:
        correct, trajectory = jointLimitsDMP1.imitateDemoInsideBounds(y_demo, x0, g)
        assert str(exception_info.value) == 'demo.rows != g.rows'
      
    g = np.zeros(2)
    time = np.linspace(0,execution_time, math.ceil(execution_time/dt))
    y_demo1 = np.sin(time*3*np.pi/2)*20*np.pi/180
    y_demo1 = np.vstack((y_demo1,-np.sin(time*3*np.pi/2)*50*np.pi/180))
    with pytest.raises(Exception) as exception_info:
        correct, trajectory = jointLimitsDMP1.imitateDemoInsideBounds(y_demo, x0, g)
        assert str(exception_info.value) == "demo.columns != execution_time/dt+1"



def test_imitateDemoInsideBounds():
    dt = 0.001
    execution_time = 1.00
    n_features = 10
    qmin = [-40,-60, -55]
    qmax = [15,65, 50]
    dmp_name = "3_joints"
    jointLimitsDMP1 = jointLimitsDMP.JointLimitsDMP(dt, execution_time, n_features, qmin, qmax, dmp_name, len(qmin)+1)
    y_demo = np.array([])


    time = np.linspace(0,execution_time, math.ceil(execution_time/dt))
    gamma = np.transpose(np.array([time, np.sin(time*3.5*np.pi/2)*15, -np.sin(time*3*np.pi/2)*50, -np.sin(time*5*np.pi/2)*30]))
    x0 = gamma[0].copy()
    g = gamma[-1].copy()
    correct, trajectory = jointLimitsDMP1.imitateDemoInsideBounds(gamma, x0, g)
    jointLimitsDMP1.plotDMPS(gamma, trajectory)
    assert correct == True
    
def test_setDt():
    print("Set new dt")
    dt = 0.001
    execution_time = 1.0
    n_features = 10
    qmin = [-40,-60, -55]
    qmax = [25,65, 50]
    dmp_name = "2_joints"
    jointLimitsDMP1 = jointLimitsDMP.JointLimitsDMP(dt, execution_time, n_features, qmin, qmax, dmp_name, len(qmin))
    
    jointLimitsDMP1.setDMPDt(0.1)
    
    assert jointLimitsDMP1.getArgs()['dt'] == 0.1

def test_setExecutionTime():
    dt = 0.001
    execution_time = 1.0
    n_features = 10
    qmin = [-40,-60, -55]
    qmax = [25,65, 50]
    dmp_name = "2_joints"
    jointLimitsDMP1 = jointLimitsDMP.JointLimitsDMP(dt, execution_time, n_features, qmin, qmax, dmp_name, len(qmin))
    
    jointLimitsDMP1.setExecutionTime(2.0)
    
    assert jointLimitsDMP1.getArgs()['execution_time'] == 2.0

def test_setNFeatures():
    dt = 0.001
    execution_time = 1.0
    n_features = 10
    qmin = [-40,-60, -55]
    qmax = [25,65, 50]
    dmp_name = "2_joints"
    jointLimitsDMP1 = jointLimitsDMP.JointLimitsDMP(dt, execution_time, n_features, qmin, qmax, dmp_name, len(qmin))
    
    jointLimitsDMP1.setNFeatures(15)
    
    assert jointLimitsDMP1.getArgs()['n_features'] == 15
    