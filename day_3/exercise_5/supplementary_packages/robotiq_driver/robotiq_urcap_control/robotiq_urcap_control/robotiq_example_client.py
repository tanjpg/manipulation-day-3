
from robotiq_msgs.srv import CModelResponse

import rclpy


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('minimal_client')
    cli = node.create_client(CModelResponse, 'gripper_service')
    
    configuration = int(input('Which gripper are you using? \nRobotiq = 1, ePick = 2: '))

    req = CModelResponse.Request()
    req.configuration = configuration

    if configuration != 2:

        robotiq2f_type = int(input('Which robotiq 2 finger gripper are you using? \n85 or 140? :'))
        req.robotiq2f_type = robotiq2f_type
        pos = int(input('Enter width of object [0-%d]: ' % (robotiq2f_type)))
        speed = int(input('Enter a speed value [0-255]: '))
        force = int(input('Enter a force value [0-255]: '))

        req.rpr = pos
        req.rsp = speed
        req.rfr = force

    else:
        suction_mode = int(input('Choose the ePick mode, \n Activate Suction Gripper = 1, Release Object = 2 : '))

        if suction_mode == 1:
            print('Suction ON')
            req.rmod = 1    # ePick advanced mode(1). Other option is automatic mode(0) but currently not used
            req.rpr = 0     # Value can be adjusted. 0[Maximum vacuum level] - 99[Minimum vacuum level]. > 100 = passive release 
            req.rsp = 0     # Timeout value. Default: 0
            req.rfr = 70    ## Acceptable pressure on workpiece, once vaccum level is reached status message gOBJ = 1.
                            # 0 - Object detected once vacuum level reaches 100,
                            # 30 - Object detected once vacuum level reaches 70,
                            # Value has to be < 100

        elif suction_mode == 2:
            print('Suction OFF')
            req.rmod = 1
            req.rpr = 255 # Value can be adjusted. 0[Maximum vacuum level] - 99[Minimum vacuum level]. > 100 = passive release
            req.rsp = 0 # Timeout value. Leave it at 0

        # Automatic Mode not required
        # else:
            # print('Gripper set in AUTOMATIC Mode')
            # req.rmod = 0
            #
            # automatic_release =  bool(input('Do you want to turn off the vacuum? True or False: '))
            #
            # if automatic_release == True:
                # req.ratr = 1


    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    result = future.result()
    node.get_logger().info(
        'Position: %d, Speed: %d, Force: %d. %s' %
        (req.rpr, req.rsp, req.rfr, result.response))

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
