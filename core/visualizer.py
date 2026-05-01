import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
    
class Visualizer:
    def __init__(self, bodies, trajectories, follow, speed, centered=False, rel_mass=False):
        self.bodies = bodies
        self.trajectories = trajectories
        self.dim = len(self.bodies[0].position)
        self.follow = follow
        self.speed = speed
        self.rel_mass = rel_mass
        self.centered = centered
        
        self.N = len(self.bodies)
        self.dim = len(self.bodies[0].position)
    
    def plotting(self, display=False):
        
        if self.dim == 3:
            fig, ax = plt.subplots(figsize=(8,8), subplot_kw={"projection":"3d"})
        else:
            fig, ax = plt.subplots()
        
        colors = ["blue", "green", "red", "cyan", "magenta","yellow", "black", "white"]
        
        self.ax = ax
        self.b_plt = []
        self.b_dot = []
        
        masitas = [b.mass for b in self.bodies]
        pan_de_canela = 5
        
        for i in range(self.N):
            c = i % len(colors)
            
            if self.rel_mass:
                pan_de_canela = masitas[i]/max(masitas) * 10
                
            body_plt, = ax.plot(*self.trajectories[i], colors[c], label=f"Body {i+1}", linewidth=1)
            
            dot = [posi[-1] for posi in self.trajectories[i]]
            body_dot, = ax.plot(*dot, 'o', color=colors[c], markersize=pan_de_canela)
            
            self.b_plt.append(body_plt)
            self.b_dot.append(body_dot)
            
        ax.set_title(f"The {self.N}-Body Problem")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        if self.dim == 3:
            ax.set_zlabel("z")
            ax.set_box_aspect([1, 1, 1])
        else:
            ax.set_box_aspect(1)

        ax.set_aspect('equal')
        plt.grid()
        plt.legend()
        
        if self.centered:
            total_mass = sum([b.mass for b in self.bodies])
            
            CG = np.sum([b.position * b.mass for b in self.bodies], axis=0) / total_mass
            
            zoom = 10
            
            xlim = abs(list(self.ax.get_xlim())[0] - list(self.ax.get_xlim())[1])/zoom
            ylim = abs(list(self.ax.get_ylim())[0] - list(self.ax.get_ylim())[1])/zoom
            
            if self.dim == 3:
                zlim = abs(list(self.ax.get_zlim())[0] - list(self.ax.get_zlim())[1])/zoom
                puta = max([zlim, ylim, xlim])
                self.ax.set_zlim(CG[2] - puta, CG[2] + puta)
            else:
                puta = max([ylim, xlim])
            
            self.ax.set_xlim(CG[0] - puta, CG[0] + puta)
            self.ax.set_ylim(CG[1] - puta, CG[1] + puta)
             
        
        if display:
            plt.show()
        return fig

    def update(self, frame):
        
        bot_lim = max(0, frame - self.follow)
        plt_dot = []
       
        xlim = list(self.ax.get_xlim())
        ylim = list(self.ax.get_ylim())
        if self.dim == 3:
            zlim = list(self.ax.get_zlim())
        
        if self.centered:
            margin = 0.05
                    
            for i in range(self.N):
                x_pos = self.trajectories[i][0][frame]
                y_pos = self.trajectories[i][1][frame]
                
                if self.dim == 3:
                    z_pos = self.trajectories[i][2][frame]
                    if z_pos < zlim[0]: zlim[0] = z_pos - margin
                    if z_pos > zlim[1]: zlim[1] = z_pos + margin
                
                if x_pos < xlim[0]: xlim[0] = x_pos - margin
                if x_pos > xlim[1]: xlim[1] = x_pos + margin
                
                if y_pos < ylim[0]: ylim[0] = y_pos - margin
                if y_pos > ylim[1]: ylim[1] = y_pos + margin
                
            self.ax.set_xlim(xlim)
            self.ax.set_ylim(ylim)
            if self.dim == 3:
                self.ax.set_zlim(zlim)
        
        for i in range(self.N):
            x_data = self.trajectories[i][0][bot_lim:frame+1]
            y_data = self.trajectories[i][1][bot_lim:frame+1]
            if self.dim == 3:
                z_data = self.trajectories[i][2][bot_lim:frame+1]
            
            self.b_plt[i].set_data(x_data, y_data)
            if self.dim == 3:
                self.b_plt[i].set_3d_properties(z_data)
            
            self.b_dot[i].set_data(x_data, y_data)
            if self.dim == 3:
                self.b_plt[i].set_3d_properties(z_data)
            
            self.b_dot[i].set_data([x_data[-1]], [y_data[-1]])
            if self.dim == 3:
                self.b_dot[i].set_3d_properties([z_data[-1]])
            
            plt_dot.append(self.b_plt[i])
            plt_dot.append(self.b_dot[i])
           
        return plt_dot
  
    def animate(self):
        frames = range(0, len(self.trajectories[0][0]), self.speed)
        self.animation = FuncAnimation(self.plotting(), self.update, frames=frames, interval=10, blit=not self.centered)
        plt.show()