import yarp

yarp.Network.init()

rpcClient = yarp.RpcClient()

rpcClient.open('/trajectoryGeneration/trunkAndRightArm/rpc:c')
yarp.Network.connect("/trajectoryGeneration/trunkAndRightArm/rpc:c",
                    "/trajectoryGeneration/trunkAndRightArm/rpc:s")

cmd = yarp.Bottle()
response = yarp.Bottle()
cmd.addVocab32("cpgp")
goal = cmd.addList()

goal.addDouble(0.598779265336311095069)
goal.addDouble(-0.35392221202090445864)
goal.addDouble(0.185557171940966963986)
goal.addDouble(-1.45157074841399014176)
goal.addDouble(0.719449224408488241522)
goal.addDouble(-0.719449224408488241522)

while True:
    rpcClient.write(cmd, response)
    print(response.toString())