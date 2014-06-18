
def saveFigTight(fig, filename):
    ax = fig.gca()
    extent = ax.get_window_extent().transformed(fig.dpi_scale_trans.inverted())
    fig.savefig(filename, bbox_inches=extent)
