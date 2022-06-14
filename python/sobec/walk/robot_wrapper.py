import pinocchio as pin
import numpy as np


def addChildrenFrames(
    model,
    parentFrameIds,
    subname,
    displacement=None,
    translation=None,
    xtranslation=None,
):
    """
    Add a child frame to all frames listed by their id.
    The child placement is given wrt to the parent by <displacement>.
    The new name is the parent name suffixed by <subname>.
    """
    assert (
        displacement is not None or translation is not None or xtranslation is not None
    )
    if displacement is None:
        if translation is None:
            translation = np.array([xtranslation, 0, 0])
        displacement = pin.SE3(np.eye(3), translation)
    for cid in parentFrameIds:
        assert cid < len(model.frames)
        f = model.frames[cid]
        newframe = pin.Frame(
            f.name + "_" + subname,
            f.parent,
            f.previousFrame,
            f.placement * displacement,
            pin.FrameType.OP_FRAME,
        )
        model.addFrame(newframe)


class RobotWrapper:
    def __init__(self, model, contactKey, refPosture="half_sitting"):
        """
        :param: a well-built pinocchio model, for example built from URDF.
        :param: contactKey: the sub-string for all frame that will act as contacts

        The following fields are provides:
        - contactIds
        - towIds and heelIds
        - x0
        - model and data
        - baseId
        - com0
        - robotGravForce
        """
        self.model = model
        self.contactIds = [
            i for i, f in enumerate(self.model.frames) if contactKey in f.name
        ]

        addChildrenFrames(self.model, self.contactIds, "tow", xtranslation=0.1)
        self.towIds = {
            idf: model.getFrameId("%s_tow" % model.frames[idf].name)
            for idf in self.contactIds
        }

        addChildrenFrames(self.model, self.contactIds, "heel", xtranslation=-0.1)
        self.heelIds = {
            idf: model.getFrameId("%s_heel" % model.frames[idf].name)
            for idf in self.contactIds
        }

        # The pinocchio model is what we are really interested by.
        q0 = self.model.referenceConfigurations[refPosture]
        self.x0 = np.concatenate([q0, np.zeros(self.model.nv)])
        self.data = self.model.createData()

        # Some key elements of the model
        self.baseId = model.getFrameId("root_joint")
        self.gravForce = (
            -sum(Y.mass for Y in self.model.inertias) * self.model.gravity.linear[2]
        )
        self.com0 = pin.centerOfMass(self.model, self.data, q0)
