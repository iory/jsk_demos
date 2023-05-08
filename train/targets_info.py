targets_shelf = [
    'alfort',
    'black_coffee',
    'boss',
    'cookie',
    'frisk',
    'haichu',
    'hojitya',
    'ippon',
    'irohasu',
    'jagariko',
    'koala',
    'others',
    'pocari',
    'ritz',
    'takenoko',
    'cone',
]

targets_industry = [
    "fan",
    "circuit",
    "BA25",
    "RAU",
    "SHIM_RINGS",
    "kinoko",
    "frisk",
    "jagariko",
    "alfort",
    "takenoko",
    "koala",
    "ippon",
    'others',
]


targets_robot = [
    'alfort',
    'cookie',
    'kinoko',
    'jagariko',
    'koala',
    "frisk",
]

targets_outdoor = [
    'cabbage',
]

targets_yamagata = [
    'fan',
    'BA25',
    'RAU',
    'D515DL',
    'S-51',
    'スポンジ研磨剤',
    '手袋',
    'USB3.0',
    'USB3.0-box',
]

targets_foreground = [
    'others',
]


targets_dspl = [
    'Bamboo shoot',
    'Chocolate',
    'Coffee',
    'Coke',
    'Green tea',
    'Noodle',
]


targets_fruit = [
    'apple',
    'banana',
    'lemon',
    'orange',
    'peach',
    'pear',
    'plum',
    'strawberry',
    'Cheez-it cracker box',
    'French’s mustard bottle',
    'Jell-o strawberry gelatin box',
    'Pringles chips can',
    'Starkist tuna fish can',
    'Domino sugar box',
    'Jell-o chocolate pudding box',
    'Master chef coffee can',
    'Spam potted meat can',
    'Tomato soup can',
]


targets_kxr = [
    'parts1',
    'parts2',
    'parts3',
    'servo',
]

targets_home = [
    'dish',
]


def get_targets(target):
    max_angle = 360
    min_scale = 0.1
    max_scale = 0.4
    if target == 'industry':
        targets = targets_industry
    elif target == 'robot':
        targets = targets_robot
    elif target == 'outdoor':
        targets = targets_outdoor
        min_scale = 0.04
        max_scale = 0.4
    elif target == 'yamagata':
        targets = targets_yamagata
        min_scale = 0.2
        max_scale = 0.6
    elif target == 'foreground':
        targets = targets_foreground
        min_scale = 0.1
        max_scale = 0.6
    elif target == 'dspl':
        targets = targets_dspl
        min_scale = 0.05
        max_scale = 0.4
    elif target == 'fruit':
        targets = targets_fruit
        min_scale = 0.1
        max_scale = 0.4
    elif target == 'kxr':
        targets = targets_kxr
        min_scale = 0.1
        max_scale = 0.4
    elif target == 'home':
        targets = targets_home
        min_scale = 0.1
        max_scale = 0.4
    else:
        targets = targets_shelf
        max_angle = 10
        min_scale = 0.04
        max_scale = 0.35
    return targets, max_angle, min_scale, max_scale
