from scuttlepy import SCUTTLE

scuttle = SCUTTLE(openLoop=True)

try:

    while True:

        scuttle.setMotion([0.4, 0])
        print(scuttle.getMotion())

except KeyboardInterrupt:

    pass

finally:

    scuttle.stop()
