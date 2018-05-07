#! /usr/bin/env python

import sys
import subprocess
import glob

args=sys.argv
if len(args)<2:
    print(
"""
Two arguments should be given:
    - a size : big, small or multiple_small.
    - if big, give a number between 0 and 249 (included)
    - if small, give a list of numbers (max=6) between 0 and 249 (included)
    - if multiple_small give the start and the end of the interval (included)
""")
    sys.exit(1)
else:
    def bash_command(cmd):
        p=subprocess.Popen(cmd, shell=True, executable='/bin/bash')
        p.wait()

    if args[1]=="multiple_small":
      start=int(args[2])
      end=int(args[3])
      l=[str(i) for i in range(start,end+1)]
      s=""
      for i in range(len(l)):
        s+=" "+l[i]
        if i%6==5 or i==len(l)-1:
          bash_command("./print_aruco.py small%s"%s)
          s=""
      out=glob.glob("aruco_small_*")
      cmd="pdftk "
      for file in out:
        cmd+=file+" "
      cmd+="cat output aruco_multiple_small_%d_%d.pdf"%(start,end)
      bash_command(cmd)

      cmd="rm aruco_small_*"
      bash_command(cmd)


      sys.exit(0)

    latex_file="""\documentclass[a4paper]{article}
    \pagenumbering{gobble}

    \usepackage[absolute]{textpos}
    \usepackage{graphicx}

    \setlength{\TPHorizModule}{1mm} %unit
    \setlength{\TPVertModule}{\TPHorizModule}%unit
    \\textblockorigin{0pt}{0pt}
    \setlength{\parindent}{0pt}

    \\begin{document}"""
    if args[1]=="big":
        size=200 #should be less than 297
        n=args[2]
        latex_file+="""
            \\begin{textblock}{3}(%f,%f)
                \includegraphics[width=%smm]{tmp/aruco_mip_36h12_%s.png}
            \end{textblock}"""%(float((210-size)/2),float((297-size)/2),size,n.zfill(5))
    elif args[1]=="small":
        size=90
        border=9
        total=size+border
        horizontal_margin=(210-2*total)/2.0
        for i in range(len(args[2:])):
            latex_file+="""\\begin{textblock}{0}(%d,%d)
                \setlength{\\fboxsep}{%dmm}%%
                \setlength{\\fboxrule}{0mm}%%
                \\fbox{%%
                    \includegraphics[width=%smm]{tmp/aruco_mip_36h12_%s.png}%%
                }%%
            \end{textblock}"""%(i%2*total+horizontal_margin,i//2*total,border/2.0,size,args[2+i].zfill(5))
    latex_file+="""\end{document}
    \endinput"""

    if args[1]=="big":
        filenametex="aruco"
    else:
        filenametex="aruco_small"
    for n in args[2:]:
        filenametex+="_"+n
    filenamepdf=filenametex+".pdf"
    filenametex+=".tex"
    f=open(filenametex,"w")
    f.write(latex_file)
    f.close()

    import time
    bash_command('mkdir tmp')
    bash_command('../../../../../devel_release/lib/aruco/aruco_print_dictionary ./tmp ARUCO_MIP_36h12 > /dev/null')
    bash_command('pdflatex -output-directory=tmp %s'%filenametex)
    bash_command('mv tmp/%s .'%filenamepdf)
    bash_command('mv %s tmp'%filenametex)
    bash_command('rm -r tmp')

    sys.exit(0)
