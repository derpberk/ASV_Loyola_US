---
layout: page
---

This was performed with drone 1




{% highlight ruby %}
1649326882.4894710 [mission-1] [INFO] [1649326882.488258494] [ASV.mission_node]: going to asv_interfaces.msg.Location(lon=-5.939990043607396, lat=37.30815862391698)
1649326882.4912174 [mqtt-4] [INFO] [1649326882.489882561] [ASV.mqtt_node]: point sent
1649326882.4976888 [drone-3] [INFO] [1649326882.496920857] [ASV.drone_node]: Action call received
1649326882.5004117 [drone-3] [INFO] [1649326882.499711266] [ASV.drone_node]: Action accepted
1649326882.5093994 [planner-6] [INFO] [1649326882.508428849] [ASV.mission_node]: calculating path
1649326882.7910244 [planner-6] [INFO] [1649326882.789352026] [ASV.mission_node]: path calculated in 0.027750433 seconds
1649326882.7980733 [mission-1] [INFO] [1649326882.797199012] [ASV.mission_node]: Goal accepted :)
1649326882.8137953 [drone-3] [INFO] [1649326882.812766612] [ASV.drone_node]: path is [asv_interfaces.msg.Location(lon=-5.940092, lat=37.3074785), asv_interfaces.msg.Location(lon=-5.9400835, lat=37.3074875), asv_interfaces.msg.Location(lon=-5.9400655, lat=37.3074875), asv_interfaces.msg.Location(lon=-5.9400565, lat=37.3074965), asv_interfaces.msg.Location(lon=-5.9400385, lat=37.3074965), asv_interfaces.msg.Location(lon=-5.9399845, lat=37.3075405), asv_interfaces.msg.Location(lon=-5.939966500000001, lat=37.307567500000005), asv_interfaces.msg.Location(lon=-5.9399575, lat=37.307576499999996), asv_interfaces.msg.Location(lon=-5.939949, lat=37.3075855), asv_interfaces.msg.Location(lon=-5.9399405000000005, lat=37.3075945), asv_interfaces.msg.Location(lon=-5.9399315, lat=37.3075945), asv_interfaces.msg.Location(lon=-5.9399225, lat=37.307612500000005), asv_interfaces.msg.Location(lon=-5.939913499999999, lat=37.307621499999996), asv_interfaces.msg.Location(lon=-5.9399045, lat=37.3076305), asv_interfaces.msg.Location(lon=-5.9398955, lat=37.3076305), asv_interfaces.msg.Location(lon=-5.9398865, lat=37.3076485), asv_interfaces.msg.Location(lon=-5.9398775, lat=37.307657500000005), asv_interfaces.msg.Location(lon=-5.9398685, lat=37.3076755), asv_interfaces.msg.Location(lon=-5.9398685, lat=37.307792500000005), asv_interfaces.msg.Location(lon=-5.9398595, lat=37.3078555), asv_interfaces.msg.Location(lon=-5.9398595, lat=37.307999499999994), asv_interfaces.msg.Location(lon=-5.9398775, lat=37.307999499999994), asv_interfaces.msg.Location(lon=-5.9399225, lat=37.3080085), asv_interfaces.msg.Location(lon=-5.939949, lat=37.308035000000004), asv_interfaces.msg.Location(lon=-5.9399845, lat=37.308151499999994)]
1649326882.8162453 [drone-3] [INFO] [1649326882.815562556] [ASV.drone_node]: Turning to : 64.62978339805987 N
1649326886.9483316 [mission-1] [INFO] [1649326886.947228286] [ASV.mission_node]: distance=14.768561826765819
1649326890.2675178 [mission-1] [INFO] [1649326890.266277701] [ASV.mission_node]: distance=13.8170527224186
1649326892.9843051 [drone-3] CRITICAL:autopilot:EKF variance
1649326892.9866929 [drone-3] CRITICAL:autopilot:EKF failsafe!
1649326893.3890896 [mission-1] [INFO] [1649326893.387888055] [ASV.mission_node]: distance=12.14030170329303
1649326896.6634004 [mission-1] [INFO] [1649326896.662281626] [ASV.mission_node]: distance=9.945559022747847
1649326897.4846582 [drone-3] CRITICAL:autopilot:EKF failsafe cleared
1649326899.8264081 [mission-1] [INFO] [1649326899.825023529] [ASV.mission_node]: distance=5.537133826950408
1649326903.0448790 [mission-1] [INFO] [1649326903.043689437] [ASV.mission_node]: distance=2.3037643329646054
1649326903.6475837 [drone-3] [INFO] [1649326903.646511188] [ASV.drone_node]: waypoint reached going to [37.3074875,-5.9400835
1649326904.9590690 [drone-3] [INFO] [1649326904.958026426] [ASV.drone_node]: waypoint reached going to [37.3074875,-5.9400655
1649326906.2736712 [mission-1] [INFO] [1649326906.272677870] [ASV.mission_node]: distance=1.533238524973213
1649326906.3737075 [drone-3] [INFO] [1649326906.372543729] [ASV.drone_node]: waypoint reached going to [37.3074965,-5.9400565
1649326907.3871140 [drone-3] [INFO] [1649326907.385955260] [ASV.drone_node]: waypoint reached going to [37.3074965,-5.9400385
1649326909.2080016 [drone-3] [INFO] [1649326909.206912295] [ASV.drone_node]: waypoint reached going to [37.3075405,-5.9399845
1649326909.5171654 [mission-1] [INFO] [1649326909.516162242] [ASV.mission_node]: distance=7.702744168724705
1649326912.7397013 [mission-1] [INFO] [1649326912.738584475] [ASV.mission_node]: distance=4.33335686461632
1649326915.7556529 [drone-3] [INFO] [1649326915.754571671] [ASV.drone_node]: waypoint reached going to [37.307567500000005,-5.939966500000001
1649326915.9645274 [mission-1] [INFO] [1649326915.963375944] [ASV.mission_node]: distance=4.42011624110514
1649326918.8792481 [drone-3] [INFO] [1649326918.878115018] [ASV.drone_node]: waypoint reached going to [37.307576499999996,-5.9399575
1649326919.2655995 [mission-1] [INFO] [1649326919.264631256] [ASV.mission_node]: distance=2.2628432164843697
1649326920.0972917 [drone-3] [INFO] [1649326920.096243862] [ASV.drone_node]: waypoint reached going to [37.3075855,-5.939949
1649326921.4107749 [drone-3] [INFO] [1649326921.409677411] [ASV.drone_node]: waypoint reached going to [37.3075945,-5.9399405000000005
1649326922.4231606 [mission-1] [INFO] [1649326922.422086412] [ASV.mission_node]: distance=1.5125889927755691
1649326922.6232736 [drone-3] [INFO] [1649326922.622159711] [ASV.drone_node]: waypoint reached going to [37.3075945,-5.9399315
1649326923.2322586 [drone-3] [INFO] [1649326923.231117394] [ASV.drone_node]: waypoint reached going to [37.307612500000005,-5.9399225
1649326925.1480312 [drone-3] [INFO] [1649326925.146683310] [ASV.drone_node]: waypoint reached going to [37.307621499999996,-5.939913499999999
1649326925.6636961 [mission-1] [INFO] [1649326925.662045749] [ASV.mission_node]: distance=2.128180658363898
1649326926.3620481 [drone-3] [INFO] [1649326926.361001595] [ASV.drone_node]: waypoint reached going to [37.3076305,-5.9399045
1649326927.8737576 [drone-3] [INFO] [1649326927.872663154] [ASV.drone_node]: waypoint reached going to [37.3076305,-5.9398955
1649326928.6827288 [drone-3] [INFO] [1649326928.681665458] [ASV.drone_node]: waypoint reached going to [37.3076485,-5.9398865
1649326928.8899426 [mission-1] [INFO] [1649326928.888725429] [ASV.mission_node]: distance=3.082219149204928
1649326930.9021151 [drone-3] [INFO] [1649326930.901014937] [ASV.drone_node]: waypoint reached going to [37.307657500000005,-5.9398775
1649326932.1643214 [mission-1] [INFO] [1649326932.163202613] [ASV.mission_node]: distance=1.6197761311717103
1649326932.2168343 [drone-3] [INFO] [1649326932.214988359] [ASV.drone_node]: waypoint reached going to [37.3076755,-5.9398685
1649326934.6357327 [drone-3] [INFO] [1649326934.634090586] [ASV.drone_node]: waypoint reached going to [37.307792500000005,-5.9398685
1649326935.3483160 [mission-1] [INFO] [1649326935.346798703] [ASV.mission_node]: distance=13.649784052497775
1649326938.6643381 [mission-1] [INFO] [1649326938.662346234] [ASV.mission_node]: distance=10.945450533435485
1649326941.7851293 [mission-1] [INFO] [1649326941.783571650] [ASV.mission_node]: distance=8.629613712077822
1649326945.0046990 [mission-1] [INFO] [1649326945.003018383] [ASV.mission_node]: distance=6.077205961119177
1649326948.2658782 [mission-1] [INFO] [1649326948.263785436] [ASV.mission_node]: distance=2.660756119804
1649326949.3258164 [drone-3] [INFO] [1649326949.324812271] [ASV.drone_node]: waypoint reached going to [37.3078555,-5.9398595
1649326951.4459267 [mission-1] [INFO] [1649326951.444548366] [ASV.mission_node]: distance=6.527474962098541
1649326954.6691787 [mission-1] [INFO] [1649326954.666760403] [ASV.mission_node]: distance=3.686570100219319
1649326956.8802619 [drone-3] [INFO] [1649326956.879074518] [ASV.drone_node]: waypoint reached going to [37.307999499999994,-5.9398595
1649326957.8918300 [mission-1] [INFO] [1649326957.890669393] [ASV.mission_node]: distance=16.54200169607494
1649326961.1649165 [mission-1] [INFO] [1649326961.163495842] [ASV.mission_node]: distance=13.84807564214969
1649326964.3385561 [mission-1] [INFO] [1649326964.337326444] [ASV.mission_node]: distance=11.510858952314596
1649326967.5563519 [mission-1] [INFO] [1649326967.555170673] [ASV.mission_node]: distance=8.583235555563682
1649326970.7749221 [mission-1] [INFO] [1649326970.773739548] [ASV.mission_node]: distance=5.332172675234581
1649326973.9962347 [mission-1] [INFO] [1649326973.995065725] [ASV.mission_node]: distance=2.182729498494875
1649326974.5993533 [drone-3] [INFO] [1649326974.598129532] [ASV.drone_node]: waypoint reached going to [37.307999499999994,-5.9398775
1649326976.2131641 [drone-3] [INFO] [1649326976.211870956] [ASV.drone_node]: waypoint reached going to [37.3080085,-5.9399225
1649326977.2663774 [mission-1] [INFO] [1649326977.265118148] [ASV.mission_node]: distance=4.387395103726393
1649326980.4546506 [mission-1] [INFO] [1649326980.453524329] [ASV.mission_node]: distance=2.1466763530717152
1649326981.3578494 [drone-3] [INFO] [1649326981.356805282] [ASV.drone_node]: waypoint reached going to [37.308035000000004,-5.939949
1649326983.6784172 [mission-1] [INFO] [1649326983.676904551] [ASV.mission_node]: distance=3.1384434148991915
1649326985.3861108 [drone-3] [INFO] [1649326985.385024199] [ASV.drone_node]: waypoint reached going to [37.308151499999994,-5.9399845
1649326986.9022305 [mission-1] [INFO] [1649326986.901081856] [ASV.mission_node]: distance=13.034728242446477
1649326990.1648059 [mission-1] [INFO] [1649326990.163452159] [ASV.mission_node]: distance=10.08186682615316
1649326993.3396730 [mission-1] [INFO] [1649326993.338601348] [ASV.mission_node]: distance=6.411565233917827
1649326996.5546770 [mission-1] [INFO] [1649326996.553514609] [ASV.mission_node]: distance=2.6491696434673435
1649326999.7704809 [mission-1] [INFO] [1649326999.769365421] [ASV.mission_node]: distance=3.2311688980158375
1649327002.3847880 [drone-3] [INFO] [1649327002.383575814] [ASV.drone_node]: Goal reached, waiting for sample
1649327002.3913810 [sensors-5] [INFO] [1649327002.390444362] [ASV.sensors_node]: Sample requested
1649327003.3955293 [sensors-5] [INFO] [1649327003.394562492] [ASV.sensors_node]: Taking sample 1 of 1
1649327003.8996968 [sensors-5] [INFO] [1649327003.898807867] [ASV.sensors_node]: sensor read: ['\x06', '6D2FBC32C8913E4F', 'SW1', '11', 'BAT:100', 'WT:17.00', 'PH:8.57', 'DO:-5.7', 'COND:751.1', 'ORP:0.040']
1649327004.0081618 [mqtt-4] [INFO] [1649327004.007090800] [ASV.mqtt_node]: sensor data sent to database{"veh_num": 1, "date": "2022-04-07 12:23:23.901569", "Latitude": 37.3081453, "Longitude": -5.9399853, "ph": 8.57, "smart_water_battery": 100.0, "Disolved_Oxygen": -5.7, "temperature": 17.0, "conductivity": 751.1, "oxidation_reduction_potential": 0.04}
1649327004.5247147 [mission-1] [INFO] [1649327004.523442175] [ASV.mission_node]: Goal succeeded! Result: True
1649327027.3033431 [mqtt-4] [INFO] [1649327027.302254401] [ASV.mqtt_node]: Received {'mission_type': 'SIMPLEPOINT', 'lat': 37.307675779921915, 'lon': -5.940196719084009, 'VehicleNum': 1} on topic veh1
1649327027.3814266 [mqtt-4] [INFO] [1649327027.380562164] [ASV.mqtt_node]: mqtt_node is calling service change_mission_mode
1649327027.3901045 [mqtt-4] [INFO] [1649327027.389274403] [ASV.mqtt_node]: mqtt_node is calling service new_samplepoint
1649327027.4849513 [mission-1] [INFO] [1649327027.483830629] [ASV.mission_node]: new waypoint received: asv_interfaces.msg.Location(lon=-5.940196719084009, lat=37.307675779921915)
1649327027.4897580 [mission-1] [INFO] [1649327027.488397139] [ASV.mission_node]: going to asv_interfaces.msg.Location(lon=-5.940196719084009, lat=37.307675779921915)
1649327027.4922481 [mqtt-4] [INFO] [1649327027.491295854] [ASV.mqtt_node]: point sent
1649327027.4977922 [drone-3] [INFO] [1649327027.497025068] [ASV.drone_node]: Action call received
1649327027.5003555 [drone-3] [INFO] [1649327027.499736574] [ASV.drone_node]: Action accepted
1649327027.5098617 [planner-6] [INFO] [1649327027.509168597] [ASV.mission_node]: calculating path
1649327027.7997448 [mission-1] [INFO] [1649327027.798608725] [ASV.mission_node]: Goal accepted :)
1649327028.1007302 [planner-6] [INFO] [1649327028.099754090] [ASV.mission_node]: path calculated in 0.958695969 seconds
1649327028.1198912 [drone-3] [INFO] [1649327028.118687735] [ASV.drone_node]: path is [asv_interfaces.msg.Location(lon=-5.9400205, lat=37.307999499999994), asv_interfaces.msg.Location(lon=-5.9400745, lat=37.3078285), asv_interfaces.msg.Location(lon=-5.9400745, lat=37.307792500000005), asv_interfaces.msg.Location(lon=-5.9399575, lat=37.307756499999996), asv_interfaces.msg.Location(lon=-5.9399315, lat=37.307747500000005), asv_interfaces.msg.Location(lon=-5.9399045, lat=37.3077385), asv_interfaces.msg.Location(lon=-5.9398685, lat=37.307702500000005), asv_interfaces.msg.Location(lon=-5.9398685, lat=37.307666499999996), asv_interfaces.msg.Location(lon=-5.9398955, lat=37.3076305), asv_interfaces.msg.Location(lon=-5.939913499999999, lat=37.307612500000005), asv_interfaces.msg.Location(lon=-5.9399405000000005, lat=37.3075945), asv_interfaces.msg.Location(lon=-5.939949, lat=37.3075855), asv_interfaces.msg.Location(lon=-5.939949, lat=37.307576499999996), asv_interfaces.msg.Location(lon=-5.9399845, lat=37.3075405), asv_interfaces.msg.Location(lon=-5.9400385, lat=37.3074965), asv_interfaces.msg.Location(lon=-5.9400655, lat=37.3074875), asv_interfaces.msg.Location(lon=-5.940092, lat=37.3074785), asv_interfaces.msg.Location(lon=-5.9401455, lat=37.3074785), asv_interfaces.msg.Location(lon=-5.9401815, lat=37.3074965), asv_interfaces.msg.Location(lon=-5.9401995, lat=37.307522500000005), asv_interfaces.msg.Location(lon=-5.9402085, lat=37.3075405), asv_interfaces.msg.Location(lon=-5.9402085, lat=37.3076305), asv_interfaces.msg.Location(lon=-5.9401905, lat=37.307666499999996)]
1649327028.1227381 [drone-3] [INFO] [1649327028.121957356] [ASV.drone_node]: Turning to : 196.79709909648398 N
1649327032.2657785 [mission-1] [INFO] [1649327032.264707616] [ASV.mission_node]: distance=17.34846997142732
1649327035.4630837 [mission-1] [INFO] [1649327035.461924351] [ASV.mission_node]: distance=15.581473407218773
1649327037.2769091 [drone-3] CRITICAL:autopilot:EKF variance
1649327037.2801518 [drone-3] CRITICAL:autopilot:EKF failsafe!
1649327038.6794770 [mission-1] [INFO] [1649327038.678267546] [ASV.mission_node]: distance=18.217759891401283
1649327041.8763208 [drone-3] CRITICAL:autopilot:EKF failsafe cleared
1649327041.8961909 [mission-1] [INFO] [1649327041.894903549] [ASV.mission_node]: distance=9.050845770284875
1649327045.1638052 [mission-1] [INFO] [1649327045.162505261] [ASV.mission_node]: distance=3.3209143019518312
1649327047.3214908 [drone-3] [INFO] [1649327047.320478709] [ASV.drone_node]: waypoint reached going to [37.3078285,-5.9400745
1649327048.3380167 [mission-1] [INFO] [1649327048.336967416] [ASV.mission_node]: distance=18.36551036089319
1649327051.5538487 [mission-1] [INFO] [1649327051.552763555] [ASV.mission_node]: distance=16.974575788036997
1649327054.7739346 [mission-1] [INFO] [1649327054.772777368] [ASV.mission_node]: distance=15.165007038316862
1649327057.9914932 [mission-1] [INFO] [1649327057.990412526] [ASV.mission_node]: distance=12.550138906352927
1649327061.2660980 [mission-1] [INFO] [1649327061.264997712] [ASV.mission_node]: distance=10.149795802052727
1649327064.4306202 [mission-1] [INFO] [1649327064.429485232] [ASV.mission_node]: distance=7.516143534597917
1649327067.6631513 [mission-1] [INFO] [1649327067.661840478] [ASV.mission_node]: distance=4.662077051403802
1649327070.8656526 [mission-1] [INFO] [1649327070.864594104] [ASV.mission_node]: distance=1.9524716562855404
1649327071.6688929 [drone-3] [INFO] [1649327071.667629004] [ASV.drone_node]: waypoint reached going to [37.307792500000005,-5.9400745
1649327074.1637759 [mission-1] [INFO] [1649327074.162334848] [ASV.mission_node]: distance=3.184716094480489
1649327076.2009220 [drone-3] [INFO] [1649327076.199799860] [ASV.drone_node]: waypoint reached going to [37.307756499999996,-5.9399575
1649327077.3143537 [mission-1] [INFO] [1649327077.312974281] [ASV.mission_node]: distance=11.123707390633733
1649327080.5345769 [mission-1] [INFO] [1649327080.533457973] [ASV.mission_node]: distance=8.361918487678661
1649327083.7506175 [mission-1] [INFO] [1649327083.749546424] [ASV.mission_node]: distance=6.432974094614774
1649327086.9660401 [mission-1] [INFO] [1649327086.964932921] [ASV.mission_node]: distance=3.5426356734908384
1649327089.6787457 [drone-3] [INFO] [1649327089.677740458] [ASV.drone_node]: waypoint reached going to [37.307747500000005,-5.9399315
1649327090.2650635 [mission-1] [INFO] [1649327090.263875187] [ASV.mission_node]: distance=3.4706041991252676
1649327092.3973222 [drone-3] [INFO] [1649327092.396153970] [ASV.drone_node]: waypoint reached going to [37.3077385,-5.9399045
1649327093.4105504 [mission-1] [INFO] [1649327093.409432105] [ASV.mission_node]: distance=2.9535949467354623
1649327095.2180886 [drone-3] [INFO] [1649327095.216736519] [ASV.drone_node]: waypoint reached going to [37.307702500000005,-5.9398685
1649327096.6638570 [mission-1] [INFO] [1649327096.662603222] [ASV.mission_node]: distance=5.27475161525833
1649327099.8528583 [mission-1] [INFO] [1649327099.851714198] [ASV.mission_node]: distance=2.3814563426824757
1649327100.8563473 [drone-3] [INFO] [1649327100.855196113] [ASV.drone_node]: waypoint reached going to [37.307666499999996,-5.9398685
1649327103.1661978 [mission-1] [INFO] [1649327103.165153281] [ASV.mission_node]: distance=3.2517159713737365
1649327105.3864887 [drone-3] [INFO] [1649327105.385468528] [ASV.drone_node]: waypoint reached going to [37.3076305,-5.9398955
1649327106.2975426 [mission-1] [INFO] [1649327106.296458790] [ASV.mission_node]: distance=4.977843500055931
1649327109.5118637 [mission-1] [INFO] [1649327109.510775675] [ASV.mission_node]: distance=2.1028325677787763
1649327110.1143384 [drone-3] [INFO] [1649327110.112932982] [ASV.drone_node]: waypoint reached going to [37.307612500000005,-5.939913499999999
1649327112.7393577 [mission-1] [INFO] [1649327112.738103537] [ASV.mission_node]: distance=1.7063328735288141
1649327113.1400998 [drone-3] [INFO] [1649327113.138845475] [ASV.drone_node]: waypoint reached going to [37.3075945,-5.9399405000000005
1649327115.9604936 [mission-1] [INFO] [1649327115.959032996] [ASV.mission_node]: distance=1.9085558357019414
1649327116.3617065 [drone-3] [INFO] [1649327116.360685538] [ASV.drone_node]: waypoint reached going to [37.3075855,-5.939949
1649327117.6730790 [drone-3] [INFO] [1649327117.672000244] [ASV.drone_node]: waypoint reached going to [37.307576499999996,-5.939949
1649327118.6825504 [drone-3] [INFO] [1649327118.681417995] [ASV.drone_node]: waypoint reached going to [37.3075405,-5.9399845
1649327119.2672999 [mission-1] [INFO] [1649327119.265632650] [ASV.mission_node]: distance=5.787526146543255
1649327122.4106925 [mission-1] [INFO] [1649327122.409534017] [ASV.mission_node]: distance=3.3650714888719135
1649327124.3207469 [drone-3] [INFO] [1649327124.319559901] [ASV.drone_node]: waypoint reached going to [37.3074965,-5.9400385
1649327125.6631913 [mission-1] [INFO] [1649327125.661948188] [ASV.mission_node]: distance=5.867424150770251
1649327128.8486009 [mission-1] [INFO] [1649327128.847191972] [ASV.mission_node]: distance=1.8962402294033402
1649327129.3494761 [drone-3] [INFO] [1649327129.348301836] [ASV.drone_node]: waypoint reached going to [37.3074875,-5.9400655
1649327132.1650743 [mission-1] [INFO] [1649327132.163630638] [ASV.mission_node]: distance=1.7266856492077745
1649327132.4694343 [drone-3] [INFO] [1649327132.468412682] [ASV.drone_node]: waypoint reached going to [37.3074785,-5.940092
1649327135.2901895 [mission-1] [INFO] [1649327135.288844705] [ASV.mission_node]: distance=1.5496045229505608
1649327135.3896575 [drone-3] [INFO] [1649327135.388648919] [ASV.drone_node]: waypoint reached going to [37.3074785,-5.9401455
1649327138.5129912 [mission-1] [INFO] [1649327138.511337419] [ASV.mission_node]: distance=3.462631664075228
1649327140.7197213 [drone-3] [INFO] [1649327140.718435625] [ASV.drone_node]: waypoint reached going to [37.3074965,-5.9401815
1649327141.7314463 [mission-1] [INFO] [1649327141.730438424] [ASV.mission_node]: distance=3.9564233276277636
1649327144.6433737 [drone-3] [INFO] [1649327144.642306889] [ASV.drone_node]: waypoint reached going to [37.307522500000005,-5.9401995
1649327144.9529796 [mission-1] [INFO] [1649327144.951854307] [ASV.mission_node]: distance=4.2900798124585515
1649327148.1678004 [drone-3] [INFO] [1649327148.166671218] [ASV.drone_node]: waypoint reached going to [37.3075405,-5.9402085
1649327148.2675428 [mission-1] [INFO] [1649327148.266433440] [ASV.mission_node]: distance=3.5228125519702234
1649327150.3829052 [drone-3] [INFO] [1649327150.381574973] [ASV.drone_node]: waypoint reached going to [37.3076305,-5.9402085
1649327151.3937631 [mission-1] [INFO] [1649327151.392588811] [ASV.mission_node]: distance=10.190941432430304
1649327154.6636298 [mission-1] [INFO] [1649327154.662519132] [ASV.mission_node]: distance=7.493197116161597
1649327157.8221736 [mission-1] [INFO] [1649327157.821144878] [ASV.mission_node]: distance=3.9965689097201107
1649327160.6362045 [drone-3] [INFO] [1649327160.635051466] [ASV.drone_node]: waypoint reached going to [37.307666499999996,-5.9401905
1649327161.0442853 [mission-1] [INFO] [1649327161.043064214] [ASV.mission_node]: distance=5.413929847080302
1649327164.2649505 [mission-1] [INFO] [1649327164.263836929] [ASV.mission_node]: distance=2.5436650531776523
1649327165.3653471 [drone-3] [INFO] [1649327165.364371204] [ASV.drone_node]: Goal reached, waiting for sample
1649327165.3712394 [sensors-5] [INFO] [1649327165.370378887] [ASV.sensors_node]: Sample requested
1649327166.3759041 [sensors-5] [INFO] [1649327166.374895181] [ASV.sensors_node]: Taking sample 1 of 1
1649327166.8798709 [sensors-5] [INFO] [1649327166.878997296] [ASV.sensors_node]: sensor read: ['\x06', '6D2FBC32C8913E4F', 'SW1', '20', 'BAT:100', 'WT:17.23', 'PH:9.06', 'DO:-5.7', 'COND:934.0', 'ORP:-0.267']
1649327166.9883842 [mqtt-4] [INFO] [1649327166.987456003] [ASV.mqtt_node]: sensor data sent to database{"veh_num": 1, "date": "2022-04-07 12:26:06.881496", "Latitude": 37.3076627, "Longitude": -5.9401907, "ph": 9.06, "smart_water_battery": 100.0, "Disolved_Oxygen": -5.7, "temperature": 17.23, "conductivity": 934.0, "oxidation_reduction_potential": -0.267}
1649327167.5035741 [mission-1] [INFO] [1649327167.502345992] [ASV.mission_node]: Goal succeeded! Result: True
1649327196.4670033 [mqtt-4] [INFO] [1649327196.465874370] [ASV.mqtt_node]: Received {'mission_type': 'MANUAL', 'VehicleNum': 1} on topic veh1
1649327196.4867795 [mission-1] [INFO] [1649327196.485578981] [ASV.mission_node]: Changed current ASV mode to MANUAL.


{% endhighlight %}