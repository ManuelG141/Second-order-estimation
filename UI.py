from tkinter import filedialog
from tkinter import *
from matplotlib.figure import Figure
from scipy.io import loadmat
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from metodos import estimateModel as es

def main():
	root = Tk()
	gui = Window(root)
	gui.root.mainloop()
	return None

class Window():
	def __init__(self, root):
		self.root = root
		self.root.title("Automatic Control P3")
		self.root.geometry('1500x800')
		self.root.minsize(width=1500, height=800)

		self.__t = 0
		self.__y = 0
		self.__model = 0

		self.cont = 0

		self.realTf = Label(self.root, text= "")
		self.estimateTF = Label(self.root, text= "")
		self.ISEpercentage = Label(self.root, text= "")
		self.realTf.place(x=100, y=60)
		self.estimateTF.place(x=100, y=120)
		self.ISEpercentage.place(x=100, y=90)

		self.loadDataButton = Button(self.root, text="Import File", command=self.loadData, width = 8, height = 2)
		self.loadDataButton.place(x=100, y=10)

	#This Method update the value of t & y from a given file
	def loadData(self):
		# Open a file dialog to select a file
		file_path = filedialog.askopenfilename(filetypes=[]) 
		
		data = loadmat(file_path)
		self.__t = np.array(data['t'].ravel().tolist())
		self.__y = np.array(data['y'].ravel().tolist())

		self.__model = es(self.__t, self.__y)

		self.updateText()
		self.graphModel()

	def updateText(self):
		self.realTf.config(text="TF = ..")
		self.estimateTF.config(text=f"Estimate TF: \n {self.__model.getTransferFunc()}")
		self.ISEpercentage.config(text=f"ISE: {self.__model.getErrorIndex()*100}%")

	def graphModel(self):
		self.fig, self.axs = plt.subplots(1,2, dpi=100, figsize=(15,5))
		self.fig.suptitle("FUNCION 1")
		self.axs[0].plot(self.__t,self.__model.getStepResponse())
		self.axs[0].set_title("FUNCION ORIGINAL")

		if self.cont!=0:
			plt.close("all")
			self.canvas.get_tk_widget().destroy()
		else:
			self.cont=self.cont+1
		fig = Figure(figsize=(9, 9), dpi=90)
		subplot = fig.add_subplot(111)
		subplot.plot(self.__t, self.__y, label="Real")
		subplot.plot(self.__t, self.__model.getStepResponse(), label="Estimation")
		subplot.set_xlabel("t")
		subplot.set_ylabel("yt")
		subplot.set_title("Step Response")
		subplot.legend()

		self.canvas = FigureCanvasTkAgg(fig, master=self.root)
		self.canvas.draw()
		self.canvas.get_tk_widget().place(x=300, y=0)
	  
main()


