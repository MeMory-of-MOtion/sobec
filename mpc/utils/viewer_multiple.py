import pinocchio as pin
import numpy as np

class GepettoGhostViewer:
    def __init__(self,model,collision_model,visual_model,alpha,sceneName='ghost'):
        Viz = pin.visualize.GepettoVisualizer
        viz = Viz(model, collision_model, visual_model)
        viz.initViewer(sceneName=sceneName)
        viz.loadViewerModel()

        self.viz = viz
        self.setTransparency(alpha)
        self.viewer = viz.viewer
    def setTransparency(self,alpha):
        for g in self.viz.visual_model.geometryObjects:
            node = self.viz.getViewerNodeName(g,pin.GeometryType.VISUAL)
            self.viz.viewer.gui.setFloatProperty(node,'Alpha',alpha)
    def display(self,q):
        self.viz.display(q)
    def play(self,qs,dt):
        self.viz.play(qs,dt)
    def hide(self):
        self.viz.displayVisuals(False)
    def show(self):
        self.viz.displayVisuals(True)

class GepettoMultipleVisualizers:
    def __init__(self, model, collision_model, visual_model, nghosts, alphas=None):
        if alphas is None:
            # alphas = [1.] + [ .3 - .2/(nghosts-2)*n for n in range(nghosts-1) ]
            alphas = [1.0] + [0.5 - 0.2 / (nghosts - 2) * n for n in range(nghosts - 1)]
        print(alphas)
        assert len(alphas) == nghosts
        for a in alphas:
            assert a > 0 and a <= 1

        self.vizs = []
        Viz = pin.visualize.GepettoVisualizer
        for n in range(nghosts):
            viz = Viz(model, collision_model, visual_model)
            if n > 0:
                viz.initViewer(sceneName="ghost%s" % n)
            else:
                viz.initViewer()
            viz.loadViewerModel()

            for g in visual_model.geometryObjects:
                node = viz.getViewerNodeName(g, pin.GeometryType.VISUAL)
                viz.viewer.gui.setFloatProperty(node, "Alpha", alphas[n])

            self.vizs.append(viz)

    def display(self, qs):
        assert len(qs) == len(self.vizs)
        for q, viz in zip(qs, self.vizs):
            viz.display(q)

    def subdisplay(self, qs):
        """Where qs does not have the right length"""
        idxs = np.linspace(0, len(qs) - 1, len(self.vizs), endpoint=False, dtype=int)
        qs = [qs[int(t)] for t in idxs]
        self.display(qs)
        return qs

    def kill(self, n):
        if n == "all":
            for i in reversed(range(1, len(self.vizs))):
                self.kill(i)
        sc = self.vizs[n].sceneName
        self.vizs[n].viewer.gui.deleteNode(sc, True)
        self.vizs.remove(self.vizs[n])

    def hide(self):
        [viz1.displayVisuals(False) for viz1 in self.vizs[1:]]

    def show(self):
        [viz1.displayVisuals(True) for viz1 in self.vizs[1:]]


if __name__ == "__main__":
    import time
    import talos_low

    urdf = talos_low.load()

    gviz = GepettoMultipleVisualizers(
        urdf.model, urdf.collision_model, urdf.visual_model, 4
    )
    gviz.display(
        [urdf.q0 + np.array([n] + [0] * (urdf.model.nq - 1)) for n in range(4)]
    )

    for i in reversed(range(1, 4)):
        time.sleep(2)
        gviz.kill(i)
