from scuttlepy import SCUTTLE

# scuttle = SCUTTLE(config='/home/pi/scuttlepy/config/scuttle_default_config.yaml', openLoop=True)
scuttle = SCUTTLE()

try:

    while True:

        scuttle.setMotion([0.4, 0])
        # print(scuttle.getMotion())

except KeyboardInterrupt:

    pass

finally:

    scuttle.stop()
