import matplotlib.pyplot as plt
for x in range(10):
    fig = plt.figure(10)
    plt.plot(range(x, x+10))
    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.pause(0.3)
    fig.clear()
