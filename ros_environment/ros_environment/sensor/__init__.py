# import numpy as np
#
# observation_poses = [[0.6256430011229036, -1.088274695191722, 0.8150115238730467,
#                       -0.556972605416608, -1.2697299800668822, -1.2636620115968498],
#                      [1.6474145020975832, -0.8894636803719612, 0.26998081592643197,
#                       -0.5175225168303673, -1.8797342871850233, -2.1480065911257515],
#                      [2.38213511174542, -2.4477708291017373, 2.1311728898030613,
#                       -1.651537227290979, -2.2714103374977785, -3.7251899791354384],
#                      # [2.381727612264331, -2.4472924222774437, 2.1307195056609016,
#                      #  -1.6507905772917746, -2.271176482952192, 2.5584120835284883]
#                      ]
# observation_poses = np.array(observation_poses)
# print(observation_poses.shape)
#
# mins = np.min(observation_poses, axis=0) / np.pi * 180
# print(mins)
# maxs = np.max(observation_poses, axis=0) / np.pi * 180
# print(maxs)
# print((mins + maxs) / 2)
# print(maxs - mins)
