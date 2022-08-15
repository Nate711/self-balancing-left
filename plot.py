# IMPORT PACKAGES
import numpy as np
import pandas as pd
import pdb
from numpy import sin, cos
import plotly.graph_objects as go

if __name__ == "__main__":

    # IMPORT MG OUTPUT
    cols = "t sec, x m, y m, z m, qA deg, qB deg, qC deg, ME J"

    df = pd.read_csv('MGLeftDisk.1',
                     delim_whitespace=True, header=None, skiprows=4)
    cols = [c.strip() for c in cols.split(",")]
    df.columns = cols
    pd.options.plotting.backend = "plotly"

    print(df)

    fig = df.plot.line(x="t sec", y=df.columns)
    fig.show()