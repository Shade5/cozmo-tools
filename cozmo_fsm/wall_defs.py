from .worldmap import *

def make_walls():
    w36 = WallSpec(length=459., height=152, door_height=100,
                    markers={ 48 : (+1, (118.,50.)),
                              36 : (+1, (289.,50.)) },
                    doorways = [ (229.5, 81.) ])  # (center, width)

    w38 = WallSpec(length=680, height=152, door_height=100,
                   markers={ 44 : (+1, ( 70.,50.)),  # +1 = front markers
                             62 : (+1, (235.,50.)),
                             38 : (+1, (375.,50.)),
                             50 : (+1, (544.,50.)),
                             41 : (-1, ( 70.,50.)),  # -1 = back markers
                             56 : (-1, (238.,50.)),
                             80 : (-1, (379.,50.)),
                             68 : (-1, (544.,50.)) },
                   doorways = [ (155., 76.), (460., 77.) ])  # (center, width)

    w41 = WallSpec(length=620, height=152, door_height=100,
                   markers={ 41 : (+1, ( 70.,50.)),  # +1 = front markers
                             56 : (+1, (235.,50.)),
                             80 : (+1, (375.,50.)),
                             68 : (+1, (544.,50.)),
                             44 : (-1, ( 70.,50.)),  # -1 = back markers
                             62 : (-1, (238.,50.)),
                             38 : (-1, (380.,50.)),
                             50 : (-1, (544.,50.)) },
                   doorways = [ (160., 77.), (470., 77.) ])  # (center, width)

    w42 = WallSpec(length=615, height=152, door_height=100,
                   markers={ 143 : (+1, ( 70.,50.)),
                             131 : (+1, (235.,50.)),
                             119 : (+1, (375.,50.)),
                             107 : (+1, (544.,50.)),
                              76 : (-1, ( 75.,50.)),
                              73 : (-1, (242.,50.)),
                              75 : (-1, (376.,50.)),
                              42 : (-1, (545.,50.)) },
                   doorways = [(160., 77.), (460., 77.) ])

    w53 = WallSpec(length=620, height=152, door_height=100,
                   markers={ 53 : (+1, ( 70.,50.)),  # +1 = front markers
                             65 : (+1, (235.,50.)),
                             59 : (+1, (375.,50.)),
                             55 : (+1, (544.,50.)),
                             45 : (-1, ( 70.,50.)),  # -1 = back markers
                             64 : (-1, (238.,50.)),
                             37 : (-1, (380.,50.)),
                             67 : (-1, (544.,50.)) },
                   doorways = [ (160., 77.), (470., 77.) ])  # (center, width)

    w57 = WallSpec(length = 460, height=152, door_height=100,
                   markers = {74: (+1, (146., 50.)),
                              77: (+1, (317., 50.)),
                              57: (-1, (146., 50.)),
                              69: (-1, (317., 50.))},
                   doorways = [ (230, 77) ])

    w90 = WallSpec(length=611, height=152, door_height=100,
                   markers={ 150 : (+1, ( 68.,50.)),
                             138 : (+1, (234.,50.)),
                             126 : (+1, (364.,50.)),
                             114 : (+1, (539.,50.)),
                             102 : (-1, ( 71.,50.)),
                              90 : (-1, (239.,50.)),
                             153 : (-1, (375.,50.)),
                             141 : (-1, (544.,50.)) },
                   doorways = [ (245.5, 76.), (456., 76.) ])

    w95 = WallSpec(length=506, height=152, door_height=100,
                   markers={ 95 : (+1, ( 30.,50.)),
                             83 : (+1, (200.,50.)),
                            146 : (+1, (260.,50.)),
                            134 : (+1, (428.,50.)),
                            122 : (-1, ( 30.,50.)),
                             98 : (-1, (198.,50.)),
                            110 : (-1, (268.,50.)),
                             86 : (-1, (430.,50.)) },
                   doorways = [ (115., 77.), (344., 77.) ])

    w89 = WallSpec(length=460., height=152, door_height=100,
                    markers={ 149 : (+1, (34.,50.)),
                              125 : (+1, (194.,50.)),
                              113 : (+1, (257.,50.)),
                              137 : (+1, (428.,50.)),
                              101 : (-1, (33.,50.)),
                              89 : (-1, (204.,50.)),
                              104 : (-1, (263.,50.)),
                              92 : (-1, (428.,50.)),},
                    doorways = [ (116.5, 75.), (342., 78.) ])  # (center, width)

    w108 = WallSpec(length=460, height=152, door_height=100,
                    markers={128 : (-1, (35.,50.)),
                             140 : (-1, (205.,50.)),
                             152 : (-1, (265.,50.)),
                             116 : (-1, (435.,50.)),
                             144 : (+1, (35.,50.)),
                             132 : (+1, (205.,50.)),
                             120 : (+1, (265.,50.)),
                             108 : (+1, (435.,50.))
                             },
                    doorways = [(120.,80.),(349., 78.)])

    w166 = WallSpec(length=506, height=152, door_height=100,
                    markers={ 116 : (+1, ( 25.,50.)),
                              152 : (+1, (195.,50.)),
                              140 : (+1, (255.,50.)),
                              128 : (+1, (425.,50.)),
                              144 : (-1, ( 35.,50.)),
                              132 : (-1, (205.,50.)),
                              120 : (-1, (265.,50.)),
                              108 : (-1, (435.,50.)) },
                    doorways = [ (112., 77.), (342., 77.) ])

    # Dream House Walls
    w103 = WallSpec(length=580, height=210, door_height=105,
                    markers={ 127: (+1, (150.,50.)),
                              103: (+1, (430.,50.)) }
                    )

    w109 = WallSpec(length=450, height=210, door_height=105,
                    markers={ 109: (+1, (180.,50.)),
                              121: (+1, (340.,50.)),
                              142: (-1, (110.,50.)),
                              130: (-1, (280.,50.)) },
                    doorways = [ (260., 75.) ])

    w82  = WallSpec(length=295, height=210, door_height=105,
                    markers={ 118: (+1, (105.,50.)),
                              106: (+1, (265.,50.)),
                               82: (-1, ( 30.,50.)),
                               94: (-1, (190.,50.)) },
                    doorways = [ (185., 75.) ])

    w133 = WallSpec(length=300, height=210, door_height=105,
                    markers={ 145: (+1, ( 30.,50.)),
                              148: (+1, (200.,50.)),
                              136: (-1, (100.,50.)),
                              133: (-1, (270.,50.)) },
                    doorways = [ (115.,75.) ])

    w88  = WallSpec(length=455, height=210, door_height=105,
                    markers={ 100: (+1, (185.,50.)),
                              124: (+1, (355.,50.)),
                               88: (-1, (105.,50.)),
                              112: (-1, (270.,50.)) },
                    doorways = [ (270.,75.) ])

    w91  = WallSpec(length=840, height=210, door_height=105,
                    markers={ 115: (+1, (150.,50.)),
                               91: (+1, (400.,50.)) })

    # 12 inch walls

    w7 = WallSpec(length=610, height=190, door_width = 77, door_height=110,
                    markers={ 7  : (+1, ( 65., 50.)),
                              8  : (+1, (153.,145.)),
                              9  : (+1, (240., 50.)),
                              10 : (+1, (370., 50.)),
                              11 : (+1, (460.,145.)),
                              12 : (+1, (547., 50.)) },
                              #6  : (-1, ( 65., 50.)),
							  #5  : (-1, (153.,145.)),
							  #4  : (-1, (240., 50.)),
							  #3  : (-1, (370., 50.)),
							  #2  : (-1, (460.,145.)),
							  #1  : (-1, (547., 50.))  },
                    doorways = [ (151., 77.), (455., 77.) ])

    w13 = WallSpec(length=610, height=190, door_width = 77, door_height=110,
                    markers={ 13  : (+1, ( 65., 50.)),
                              14  : (+1, (153.,145.)),
                              15  : (+1, (241., 50.)),
                              16 : (+1, (370., 50.)),
                              17 : (+1, (460.,145.)),
                              18 : (+1, (547., 50.)) },
                              #6  : (-1, ( 65., 50.)),
                #5  : (-1, (153.,145.)),
                #4  : (-1, (240., 50.)),
                #3  : (-1, (370., 50.)),
                #2  : (-1, (460.,145.)),
                #1  : (-1, (547., 50.))  },
                    doorways = [ (151., 77.), (460., 77.) ])



make_walls()
