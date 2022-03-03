package com.gos.preseason2017.team2.robot.commands.autonomous.planning;

public class AutoRightGearLeftTalon {
    public static final int NUM_POINTS = 185;
    // Position (rotations)    Velocity (RPM)    Duration (ms)
    public static double[][] Points = new double[][] { // NOPMD(UseShortArrayInitializer)
        {0, 0, 10},
        {0.0039789, 23.873, 10},
        {0.011937, 47.746, 10},
        {0.023873, 71.62, 10},
        {0.039789, 95.493, 10},
        {0.055704, 95.493, 10},
        {0.07162, 95.493, 10},
        {0.087535, 95.493, 10},
        {0.10345, 95.493, 10},
        {0.11937, 95.493, 10},
        {0.13528, 95.493, 10},
        {0.1512, 95.493, 10},
        {0.16711, 95.493, 10},
        {0.18303, 95.493, 10},
        {0.19894, 95.493, 10},
        {0.21486, 95.493, 10},
        {0.23077, 95.493, 10},
        {0.24669, 95.493, 10},
        {0.26261, 95.493, 10},
        {0.27852, 95.493, 10},
        {0.29444, 95.493, 10},
        {0.31035, 95.493, 10},
        {0.32627, 95.493, 10},
        {0.34218, 95.493, 10},
        {0.3581, 95.493, 10},
        {0.37401, 95.493, 10},
        {0.38993, 95.493, 10},
        {0.40585, 95.493, 10},
        {0.42176, 95.493, 10},
        {0.43768, 95.493, 10},
        {0.45359, 95.493, 10},
        {0.46951, 95.493, 10},
        {0.48542, 95.493, 10},
        {0.50134, 95.493, 10},
        {0.51725, 95.493, 10},
        {0.53317, 95.493, 10},
        {0.54908, 95.493, 10},
        {0.565, 95.493, 10},
        {0.58092, 95.493, 10},
        {0.59683, 95.493, 10},
        {0.61275, 95.493, 10},
        {0.62866, 95.493, 10},
        {0.64458, 95.493, 10},
        {0.66049, 95.493, 10},
        {0.67641, 95.493, 10},
        {0.69232, 95.493, 10},
        {0.70824, 95.493, 10},
        {0.72415, 95.493, 10},
        {0.74007, 95.493, 10},
        {0.75599, 95.493, 10},
        {0.7719, 95.493, 10},
        {0.78782, 95.493, 10},
        {0.80373, 95.493, 10},
        {0.81965, 95.493, 10},
        {0.83556, 95.493, 10},
        {0.85148, 95.493, 10},
        {0.86739, 95.493, 10},
        {0.88331, 95.493, 10},
        {0.89923, 95.493, 10},
        {0.91514, 95.493, 10},
        {0.93106, 95.493, 10},
        {0.94697, 95.493, 10},
        {0.96289, 95.493, 10},
        {0.9788, 95.493, 10},
        {0.99472, 95.493, 10},
        {1.0106, 95.493, 10},
        {1.0265, 95.493, 10},
        {1.0425, 95.493, 10},
        {1.0584, 95.493, 10},
        {1.0743, 95.493, 10},
        {1.0902, 95.493, 10},
        {1.1061, 95.493, 10},
        {1.122, 95.493, 10},
        {1.138, 95.493, 10},
        {1.1539, 95.493, 10},
        {1.1698, 95.493, 10},
        {1.1857, 95.493, 10},
        {1.2016, 95.493, 10},
        {1.2175, 95.493, 10},
        {1.2335, 95.493, 10},
        {1.2494, 95.493, 10},
        {1.2653, 95.493, 10},
        {1.2812, 95.493, 10},
        {1.2971, 95.493, 10},
        {1.313, 95.493, 10},
        {1.3289, 95.493, 10},
        {1.3449, 95.493, 10},
        {1.3608, 95.493, 10},
        {1.3767, 95.493, 10},
        {1.3926, 95.493, 10},
        {1.4085, 95.493, 10},
        {1.4244, 95.493, 10},
        {1.4404, 95.493, 10},
        {1.4563, 95.493, 10},
        {1.4722, 95.493, 10},
        {1.4881, 95.493, 10},
        {1.504, 95.493, 10},
        {1.5199, 95.493, 10},
        {1.5358, 95.493, 10},
        {1.5518, 95.493, 10},
        {1.5677, 95.493, 10},
        {1.5836, 95.493, 10},
        {1.5995, 95.493, 10},
        {1.6154, 95.493, 10},
        {1.6313, 95.493, 10},
        {1.6473, 95.493, 10},
        {1.6632, 95.493, 10},
        {1.6791, 95.493, 10},
        {1.695, 95.493, 10},
        {1.7109, 95.493, 10},
        {1.7268, 95.493, 10},
        {1.7427, 95.493, 10},
        {1.7587, 95.493, 10},
        {1.7746, 95.493, 10},
        {1.7905, 95.493, 10},
        {1.8064, 95.493, 10},
        {1.8223, 95.493, 10},
        {1.8382, 95.493, 10},
        {1.8542, 95.493, 10},
        {1.8701, 95.493, 10},
        {1.886, 95.493, 10},
        {1.9019, 95.493, 10},
        {1.9178, 95.493, 10},
        {1.9337, 95.493, 10},
        {1.9496, 95.493, 10},
        {1.9656, 95.493, 10},
        {1.9815, 95.493, 10},
        {1.9974, 95.493, 10},
        {2.0133, 95.493, 10},
        {2.0292, 95.493, 10},
        {2.0451, 95.493, 10},
        {2.0611, 95.493, 10},
        {2.077, 95.493, 10},
        {2.0929, 95.493, 10},
        {2.1088, 95.493, 10},
        {2.1247, 95.493, 10},
        {2.1406, 95.493, 10},
        {2.1565, 95.493, 10},
        {2.1725, 95.493, 10},
        {2.1884, 95.493, 10},
        {2.2043, 95.493, 10},
        {2.2202, 95.493, 10},
        {2.2361, 95.493, 10},
        {2.252, 95.493, 10},
        {2.268, 95.493, 10},
        {2.2839, 95.493, 10},
        {2.2998, 95.493, 10},
        {2.3157, 95.493, 10},
        {2.3316, 95.493, 10},
        {2.3475, 95.493, 10},
        {2.3635, 95.493, 10},
        {2.3794, 95.493, 10},
        {2.3953, 95.493, 10},
        {2.4112, 95.493, 10},
        {2.4271, 95.493, 10},
        {2.443, 95.493, 10},
        {2.4589, 95.493, 10},
        {2.4749, 95.493, 10},
        {2.4908, 95.493, 10},
        {2.5067, 95.493, 10},
        {2.5226, 95.493, 10},
        {2.5385, 95.493, 10},
        {2.5544, 95.493, 10},
        {2.5704, 95.493, 10},
        {2.5863, 95.493, 10},
        {2.6022, 95.493, 10},
        {2.6181, 95.493, 10},
        {2.634, 95.493, 10},
        {2.6499, 95.493, 10},
        {2.6658, 95.493, 10},
        {2.6818, 95.493, 10},
        {2.6977, 95.493, 10},
        {2.7136, 95.493, 10},
        {2.7295, 95.493, 10},
        {2.7454, 95.493, 10},
        {2.7613, 95.493, 10},
        {2.7773, 95.493, 10},
        {2.7932, 95.493, 10},
        {2.8091, 95.493, 10},
        {2.825, 95.493, 10},
        {2.8409, 95.493, 10},
        {2.8568, 95.493, 10},
        {2.8727, 95.493, 10},
        {2.8887, 95.493, 10},
        {2.9046, 95.493, 10},
        {2.9205, 95.493, 10},
        {2.9364, 95.493, 10},
        {2.9523, 95.493, 10},
        {2.9682, 95.493, 10},
        {2.9842, 95.493, 10},
        {3.0001, 95.493, 10},
        {3.016, 95.493, 10},
        {3.0319, 95.493, 10},
        {3.0478, 95.493, 10},
        {3.0637, 95.493, 10},
        {3.0796, 95.493, 10},
        {3.0956, 95.493, 10},
        {3.1115, 95.493, 10},
        {3.1274, 95.493, 10},
        {3.1433, 95.493, 10},
        {3.1592, 95.493, 10},
        {3.1751, 95.493, 10},
        {3.1911, 95.493, 10},
        {3.207, 95.493, 10},
        {3.2229, 95.493, 10},
        {3.2388, 95.493, 10},
        {3.2547, 95.493, 10},
        {3.2706, 95.493, 10},
        {3.2865, 95.493, 10},
        {3.3025, 95.493, 10},
        {3.3184, 95.493, 10},
        {3.3343, 95.493, 10},
        {3.3502, 95.493, 10},
        {3.3661, 95.493, 10},
        {3.382, 95.493, 10},
        {3.398, 95.493, 10},
        {3.4139, 95.493, 10},
        {3.4298, 95.493, 10},
        {3.4457, 95.493, 10},
        {3.4616, 95.493, 10},
        {3.4775, 95.493, 10},
        {3.4935, 95.493, 10},
        {3.5094, 95.493, 10},
        {3.5253, 95.493, 10},
        {3.5412, 95.493, 10},
        {3.5571, 95.493, 10},
        {3.569, 71.62, 10},
        {3.581, 71.62, 10},
        {3.5929, 71.62, 10},
        {3.6049, 71.62, 10},
        {3.6168, 71.62, 10},
        {3.6287, 71.62, 10},
        {3.6407, 71.62, 10},
        {3.6526, 71.62, 10},
        {3.6645, 71.62, 10},
        {3.6765, 71.62, 10},
        {3.6884, 71.62, 10},
        {3.7004, 71.62, 10},
        {3.7123, 71.62, 10},
        {3.7242, 71.62, 10},
        {3.7362, 71.62, 10},
        {3.7481, 71.62, 10},
        {3.76, 71.62, 10},
        {3.772, 71.62, 10},
        {3.7839, 71.62, 10},
        {3.7958, 71.62, 10},
        {3.8078, 71.62, 10},
        {3.8197, 71.62, 10},
        {3.8317, 71.62, 10},
        {3.8436, 71.62, 10},
        {3.8555, 71.62, 10},
        {3.8675, 71.62, 10},
        {3.8794, 71.62, 10},
        {3.8913, 71.62, 10},
        {3.9033, 71.62, 10},
        {3.9152, 71.62, 10},
        {3.9271, 71.62, 10},
        {3.9391, 71.62, 10},
        {3.951, 71.62, 10},
        {3.963, 71.62, 10},
        {3.9749, 71.62, 10},
        {3.9868, 71.62, 10},
        {3.9988, 71.62, 10},
        {4.0107, 71.62, 10},
        {4.0226, 71.62, 10},
        {4.0346, 71.62, 10},
        {4.0465, 71.62, 10},
        {4.0585, 71.62, 10},
        {4.0704, 71.62, 10},
        {4.0823, 71.62, 10},
        {4.0943, 71.62, 10},
        {4.1062, 71.62, 10},
        {4.1181, 71.62, 10},
        {4.1301, 71.62, 10},
        {4.142, 71.62, 10},
        {4.1539, 71.62, 10},
        {4.1659, 71.62, 10},
        {4.1778, 71.62, 10},
        {4.1898, 71.62, 10},
        {4.2017, 71.62, 10},
        {4.2136, 71.62, 10},
        {4.2256, 71.62, 10},
        {4.2375, 71.62, 10},
        {4.2494, 71.62, 10},
        {4.2614, 71.62, 10},
        {4.2733, 71.62, 10},
        {4.2852, 71.62, 10},
        {4.2972, 71.62, 10},
        {4.3091, 71.62, 10},
        {4.3211, 71.62, 10},
        {4.333, 71.62, 10},
        {4.3449, 71.62, 10},
        {4.3569, 71.62, 10},
        {4.3688, 71.62, 10},
        {4.3807, 71.62, 10},
        {4.3927, 71.62, 10},
        {4.4046, 71.62, 10},
        {4.4165, 71.62, 10},
        {4.4285, 71.62, 10},
        {4.4404, 71.62, 10},
        {4.4524, 71.62, 10},
        {4.4643, 71.62, 10},
        {4.4762, 71.62, 10},
        {4.4882, 71.62, 10},
        {4.5001, 71.62, 10},
        {4.512, 71.62, 10},
        {4.524, 71.62, 10},
        {4.5359, 71.62, 10},
        {4.5479, 71.62, 10},
        {4.5598, 71.62, 10},
        {4.5717, 71.62, 10},
        {4.5837, 71.62, 10},
        {4.5956, 71.62, 10},
        {4.6075, 71.62, 10},
        {4.6195, 71.62, 10},
        {4.6314, 71.62, 10},
        {4.6433, 71.62, 10},
        {4.6553, 71.62, 10},
        {4.6672, 71.62, 10},
        {4.6792, 71.62, 10},
        {4.6911, 71.62, 10},
        {4.703, 71.62, 10},
        {4.715, 71.62, 10},
        {4.7269, 71.62, 10},
        {4.7388, 71.62, 10},
        {4.7508, 71.62, 10},
        {4.7627, 71.62, 10},
        {4.7746, 71.62, 10},
        {4.7866, 71.62, 10},
        {4.7985, 71.62, 10},
        {4.8105, 71.62, 10},
        {4.8224, 71.62, 10},
        {4.8343, 71.62, 10},
        {4.8463, 71.62, 10},
        {4.8582, 71.62, 10},
        {4.8701, 71.62, 10},
        {4.8821, 71.62, 10},
        {4.894, 71.62, 10},
        {4.906, 71.62, 10},
        {4.9179, 71.62, 10},
        {4.9298, 71.62, 10},
        {4.9418, 71.62, 10},
        {4.9537, 71.62, 10},
        {4.9656, 71.62, 10},
        {4.9776, 71.62, 10},
        {4.9895, 71.62, 10},
        {5.0014, 71.62, 10},
        {5.0174, 95.493, 10},
        {5.0333, 95.493, 10},
        {5.0492, 95.493, 10},
        {5.0651, 95.493, 10},
        {5.081, 95.493, 10},
        {5.0969, 95.493, 10},
        {5.1129, 95.493, 10},
        {5.1288, 95.493, 10},
        {5.1447, 95.493, 10},
        {5.1606, 95.493, 10},
        {5.1765, 95.493, 10},
        {5.1924, 95.493, 10},
        {5.2083, 95.493, 10},
        {5.2243, 95.493, 10},
        {5.2402, 95.493, 10},
        {5.2561, 95.493, 10},
        {5.272, 95.493, 10},
        {5.2879, 95.493, 10},
        {5.3038, 95.493, 10},
        {5.3198, 95.493, 10},
        {5.3357, 95.493, 10},
        {5.3516, 95.493, 10},
        {5.3675, 95.493, 10},
        {5.3834, 95.493, 10},
        {5.3993, 95.493, 10},
        {5.4152, 95.493, 10},
        {5.4312, 95.493, 10},
        {5.4471, 95.493, 10},
        {5.463, 95.493, 10},
        {5.4789, 95.493, 10},
        {5.4948, 95.493, 10},
        {5.5107, 95.493, 10},
        {5.5267, 95.493, 10},
        {5.5426, 95.493, 10},
        {5.5585, 95.493, 10},
        {5.5744, 95.493, 10},
        {5.5903, 95.493, 10},
        {5.6062, 95.493, 10},
        {5.6221, 95.493, 10},
        {5.6381, 95.493, 10},
        {5.654, 95.493, 10},
        {5.6699, 95.493, 10},
        {5.6858, 95.493, 10},
        {5.7017, 95.493, 10},
        {5.7176, 95.493, 10},
        {5.7336, 95.493, 10},
        {5.7495, 95.493, 10},
        {5.7654, 95.493, 10},
        {5.7813, 95.493, 10},
        {5.7932, 71.62, 10},
        {5.8052, 71.62, 10},
        {5.8171, 71.62, 10},
        {5.829, 71.62, 10},
        {5.841, 71.62, 10},
        {5.8529, 71.62, 10},
        {5.8649, 71.62, 10},
        {5.8768, 71.62, 10},
        {5.8887, 71.62, 10},
        {5.9007, 71.62, 10},
        {5.9126, 71.62, 10},
        {5.9245, 71.62, 10},
        {5.9365, 71.62, 10},
        {5.9484, 71.62, 10},
        {5.9604, 71.62, 10},
        {5.9723, 71.62, 10},
        {5.9842, 71.62, 10},
        {5.9962, 71.62, 10},
        {6.0081, 71.62, 10},
        {6.02, 71.62, 10},
        {6.032, 71.62, 10},
        {6.0439, 71.62, 10},
        {6.0558, 71.62, 10},
        {6.0678, 71.62, 10},
        {6.0797, 71.62, 10},
        {6.0917, 71.62, 10},
        {6.1036, 71.62, 10},
        {6.1155, 71.62, 10},
        {6.1275, 71.62, 10},
        {6.1394, 71.62, 10},
        {6.1513, 71.62, 10},
        {6.1633, 71.62, 10},
        {6.1752, 71.62, 10},
        {6.1871, 71.62, 10},
        {6.1991, 71.62, 10},
        {6.211, 71.62, 10},
        {6.223, 71.62, 10},
        {6.2349, 71.62, 10},
        {6.2468, 71.62, 10},
        {6.2588, 71.62, 10},
        {6.2707, 71.62, 10},
        {6.2826, 71.62, 10},
        {6.2946, 71.62, 10},
        {6.3065, 71.62, 10},
        {6.3185, 71.62, 10},
        {6.3304, 71.62, 10},
        {6.3423, 71.62, 10},
        {6.3543, 71.62, 10},
        {6.3662, 71.62, 10},
        {6.3781, 71.62, 10},
        {6.3901, 71.62, 10},
        {6.402, 71.62, 10},
        {6.4139, 71.62, 10},
        {6.4299, 95.493, 10},
        {6.4458, 95.493, 10},
        {6.4617, 95.493, 10},
        {6.4776, 95.493, 10},
        {6.4935, 95.493, 10},
        {6.5094, 95.493, 10},
        {6.5254, 95.493, 10},
        {6.5413, 95.493, 10},
        {6.5572, 95.493, 10},
        {6.5731, 95.493, 10},
        {6.589, 95.493, 10},
        {6.6049, 95.493, 10},
        {6.6208, 95.493, 10},
        {6.6368, 95.493, 10},
        {6.6527, 95.493, 10},
        {6.6686, 95.493, 10},
        {6.6845, 95.493, 10},
        {6.7004, 95.493, 10},
        {6.7163, 95.493, 10},
        {6.7323, 95.493, 10},
        {6.7482, 95.493, 10},
        {6.7641, 95.493, 10},
        {6.78, 95.493, 10},
        {6.7959, 95.493, 10},
        {6.8118, 95.493, 10},
        {6.8277, 95.493, 10},
        {6.8437, 95.493, 10},
        {6.8596, 95.493, 10},
        {6.8755, 95.493, 10},
        {6.8914, 95.493, 10},
        {6.9073, 95.493, 10},
        {6.9232, 95.493, 10},
        {6.9392, 95.493, 10},
        {6.9551, 95.493, 10},
        {6.971, 95.493, 10},
        {6.9869, 95.493, 10},
        {7.0028, 95.493, 10},
        {7.0187, 95.493, 10},
        {7.0346, 95.493, 10},
        {7.0506, 95.493, 10},
        {7.0665, 95.493, 10},
        {7.0824, 95.493, 10},
        {7.0983, 95.493, 10},
        {7.1142, 95.493, 10},
        {7.1301, 95.493, 10},
        {7.1461, 95.493, 10},
        {7.162, 95.493, 10},
        {7.1779, 95.493, 10},
        {7.1938, 95.493, 10}

    };
}
