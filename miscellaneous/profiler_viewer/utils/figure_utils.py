import csv
import numpy as np
from mpl_toolkits.mplot3d import proj3d
import matplotlib.pyplot as plt

def configure_latex_fonts():
    from matplotlib import rc
    rc('font',**{'family':'serif','serif':['Computer Modern Roman']})
    rc('text', usetex=True)
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')

#def minus_formatter(x, pos):
#    return unicode(x).replace('-', u'\u22122')

#def math_formatter(x, pos):
#    return "$%s$" % x

def math_formatter(x, pos):
    #print(x)
    #formated = "${}$".format(x)
    #if "-" in formated:
    #    print("contains minus ",formated)
    #    formated = formated.replace("-", "-")
    #    print("replaced minus ",formated)
    return x

"""
class MyLogFormatter(matplotlib.ticker.LogFormatterMathtext):
    def __call__(self, x, pos=None):
        # call the original LogFormatter
        rv = matplotlib.ticker.LogFormatterMathtext.__call__(self, x, pos)

        # check if we really use TeX
        if matplotlib.rcParams["text.usetex"]:
            # if we have the string ^{- there is a negative exponent
            # where the minus sign is replaced by the short hyphen
            rv = re.sub(r'\^\{-', r'^{\mhyphen', rv)

        return rv
"""    

def orthogonal_proj(zfront, zback):
    a = (zfront+zback)/(zfront-zback)
    b = -2*(zfront*zback)/(zfront-zback)
    # -0.0001 added for numerical stability as suggested in:
    # http://stackoverflow.com/questions/23840756
    return np.array([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,a,b],
                        [0,0,-0.0001,zback]])

def aply_orthogonal_proj():
    proj3d.persp_transformation = orthogonal_proj
    
    
def left_bottom_axis_only(axis):
    # Hide the right and top spines
    axis.spines['right'].set_visible(False)
    axis.spines['top'].set_visible(False)
    
    # Only show ticks on the left and bottom spines
    axis.yaxis.set_ticks_position('left')
    axis.xaxis.set_ticks_position('bottom')
    
    
def no_axis(axis):
    # Hide all axise spines
    axis.spines['right'].set_visible(False)
    axis.spines['top'].set_visible(False)
    axis.spines['left'].set_visible(False)
    axis.spines['bottom'].set_visible(False)
    axis.axis('off')

def set_num_y_axis_ticsk_bellow(max_num_ticks_y,axis):
    while len(axis.get_yticks()) > max_num_ticks_y:
        axis.set_yticks(axis.get_yticks()[::2])
        
def set_num_x_axis_ticsk_bellow(max_num_ticks_y,axis):
    while len(axis.get_yticks()) > max_num_ticks_y:
        axis.set_yticks(axis.get_yticks()[::2])

def circles(x, y, s, c='b', vmin=None, vmax=None, **kwargs):
    """
    Make a scatter of circles plot of x vs y, where x and y are sequence 
    like objects of the same lengths. The size of circles are in data scale.

    Parameters
    ----------
    x,y : scalar or array_like, shape (n, )
        Input data
    s : scalar or array_like, shape (n, ) 
        Radius of circle in data unit.
    c : color or sequence of color, optional, default : 'b'
        `c` can be a single color format string, or a sequence of color
        specifications of length `N`, or a sequence of `N` numbers to be
        mapped to colors using the `cmap` and `norm` specified via kwargs.
        Note that `c` should not be a single numeric RGB or RGBA sequence 
        because that is indistinguishable from an array of values
        to be colormapped. (If you insist, use `color` instead.)  
        `c` can be a 2-D array in which the rows are RGB or RGBA, however. 
    vmin, vmax : scalar, optional, default: None
        `vmin` and `vmax` are used in conjunction with `norm` to normalize
        luminance data.  If either are `None`, the min and max of the
        color array is used.
    kwargs : `~matplotlib.collections.Collection` properties
        Eg. alpha, edgecolor(ec), facecolor(fc), linewidth(lw), linestyle(ls), 
        norm, cmap, transform, etc.

    Returns
    -------
    paths : `~matplotlib.collections.PathCollection`

    Examples
    --------
    a = np.arange(11)
    circles(a, a, a*0.2, c=a, alpha=0.5, edgecolor='none')
    plt.colorbar()

    License
    --------
    This code is under [The BSD 3-Clause License]
    (http://opensource.org/licenses/BSD-3-Clause)
    """
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    from matplotlib.collections import PatchCollection

    if np.isscalar(c):
        kwargs.setdefault('color', c)
        c = None
    if 'fc' in kwargs: kwargs.setdefault('facecolor', kwargs.pop('fc'))
    if 'ec' in kwargs: kwargs.setdefault('edgecolor', kwargs.pop('ec'))
    if 'ls' in kwargs: kwargs.setdefault('linestyle', kwargs.pop('ls'))
    if 'lw' in kwargs: kwargs.setdefault('linewidth', kwargs.pop('lw'))

    patches = [Circle((x_, y_), s_) for x_, y_, s_ in np.broadcast(x, y, s)]
    collection = PatchCollection(patches, **kwargs)
    if c is not None:
        collection.set_array(np.asarray(c))
        collection.set_clim(vmin, vmax)

    ax = plt.gca()
    ax.add_collection(collection)
    ax.autoscale_view()
    if c is not None:
        plt.sci(collection)
    return collection

def loadNumberCSV(filename):
    opened = False
    print("try loading "+filename)
    with open(filename , 'rb') as csvfile:
        opened = True
        csvreader = csv.reader(csvfile)
        values = None
        for  row in csvreader:
            #print(key,row)
            row_values = []
            for value in row:
                converted = False                                                                
                if not converted:                                                                
                    try:                                                                         
                        value = int(value)                                                       
                        converted = True                                                         
                    except ValueError:                                                           
                        pass                                                                     
                if not converted:                                                                
                    try:                                                                         
                        value = float(value)                                                     
                        converted = True                                                         
                    except ValueError:                                                           
                        pass 
                row_values.append(value)
            
            to_add = np.array(row_values)        
            if(values is None):                                                                
                values = to_add                                                                
            else:                                                                                    
                values = np.vstack((values,to_add))
        return values                                                                         
    if not opened:
        print('can not read '+filename)