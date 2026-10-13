/**************************************************************************\
 * Copyright (c) Kongsberg Oil & Gas Technologies AS
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
\**************************************************************************/

#include "SoRenderManagerP.h"
#include "coindefs.h"

#include <cmath>
#include <limits>

#include <Inventor/nodes/SoInfo.h>
#include <Inventor/nodes/SoCamera.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include "Inventor/nodes/SoOrthographicCamera.h"
#include <Inventor/actions/SoGLRenderAction.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/actions/SoGLRenderAction.h>

SbBool SoRenderManagerP::touchtimer = TRUE;
SbBool SoRenderManagerP::cleanupfunctionset = FALSE;
int SoRenderManagerRootSensor::debugrootnotifications = -1;

#define PRIVATE(p) (p->pimpl)
#define PUBLIC(p) (p->publ)

#define INHERIT_TRANSPARENCY_TYPE -1

SoRenderManagerP::SoRenderManagerP(SoRenderManager * publ)
{
  this->publ = publ;
  this->getmatrixaction = NULL;
  this->getbboxaction = NULL;
  this->searchaction = NULL;
}

SoRenderManagerP::~SoRenderManagerP()
{
  delete this->getmatrixaction;
  delete this->getbboxaction;
  delete this->searchaction;
}

// Internal callback.
void
SoRenderManagerP::redrawshotTriggeredCB(void * data, SoSensor * /* sensor */)
{
#if COIN_DEBUG && 0 // debug
  SoDebugError::postInfo("SoRenderManager::redrawshotTriggeredCB", "start");
#endif // debug

  SoRenderManager * thisp = (SoRenderManager *) data;

  // Need to recheck the "active" flag, as it could have changed since
  // it was tested in the SoRenderManager::scheduleRedraw() call.
  if (PRIVATE(thisp)->isactive) { thisp->redraw(); }

#if COIN_DEBUG && 0 // debug
  SoDebugError::postInfo("SoRenderManager::redrawshotTriggeredCB", "done\n\n");
#endif // debug
}

void
SoRenderManagerP::cleanup(void)
{
  SoRenderManagerP::touchtimer = TRUE;
  SoRenderManagerP::cleanupfunctionset = FALSE;
}

void
SoRenderManagerP::updateClippingPlanesCB(void * COIN_UNUSED_ARG(closure), SoSensor * COIN_UNUSED_ARG(sensor))
{
  //SoRenderManagerP * thisp = (SoRenderManagerP *) closure;
  //if (thisp->autoclipping != SoRenderManager::NO_AUTO_CLIPPING) {
  //  thisp->setClippingPlanes();
  //}
}

void
SoRenderManagerP::setClippingPlanes(void)
{
  SoCamera * camera = this->camera;
  SoNode * scene = this->scene;
  if (!camera || !scene) return;

  SbViewportRegion vp = this->glaction->getViewportRegion();

  if (!this->getbboxaction) {
    this->getbboxaction = new SoGetBoundingBoxAction(vp);
  } else {
    this->getbboxaction->setViewportRegion(vp);
  }
  this->getbboxaction->apply(scene);

  // A float32 bounding box collapses at large absolute coordinates: the
  // float32 ULP is 64 mm at 1e9 mm, so a scene box smaller than a few
  // ULPs at its coordinate magnitude carries mostly quantization noise.
  // Driving the clipping planes from such a box would slice straight
  // through the geometry; keep the camera's current near/far instead.
  // The check must happen on the world-space box: once the box has been
  // transformed into camera space its coordinates are small again, and
  // the ULP test would use the camera-space ULP and never trigger.
  // (The empty-box fallback below is left alone: it deliberately resets
  // the planes to a sane default rather than keeping stale values.)
  {
    SbBox3f wbox = this->getbboxaction->getBoundingBox();
    if (!wbox.isEmpty()) {
      float maxabs = 0.0f;
      for (int i = 0; i < 3; i++) {
        maxabs = SbMax(maxabs, SbAbs(wbox.getMin()[i]));
        maxabs = SbMax(maxabs, SbAbs(wbox.getMax()[i]));
      }
      if (maxabs > 1.0f) {
        float wsizeX, wsizeY, wsizeZ;
        wbox.getSize(wsizeX, wsizeY, wsizeZ);
        const float wdiag = sqrtf(wsizeX * wsizeX + wsizeY * wsizeY + wsizeZ * wsizeZ);
        const float ulp = std::ldexp(1.0f, std::ilogb(maxabs) - 23);
        if (wdiag < 4.0f * ulp) {
          return;
        }
      }
    }
  }

  SbXfBox3f xbox = this->getbboxaction->getXfBoundingBox();
  SbMatrix cammat;
  SbMatrix inverse;
  this->getCameraCoordinateSystem(cammat, inverse);
  xbox.transform(inverse);

  SbMatrix mat;
  // Explicit downcast: SbMatrix / SbXfBox3f are float32 throughout;
  // this feeds only the automatic near/far computation.
  mat.setTranslate(SbVec3f(-camera->position.getValue()));
  xbox.transform(mat);
  mat = camera->orientation.getValue().inverse();
  xbox.transform(mat);
  SbBox3f box = xbox.project();

  float sizeX, sizeY, sizeZ;
  box.getSize(sizeX, sizeY, sizeZ);
  float boxDiagonal = sqrtf(sizeX * sizeX + sizeY * sizeY + sizeZ * sizeZ);

  // Clipping offset is 1% of the bounding box diagonal or at most 1.0 and at least std::numeric_limits<float>::epsilon()
  float clippingOffset = SbMin(1.0f, SbMax(std::numeric_limits<float>::epsilon(), 0.01f * boxDiagonal));

  float nearval;
  float farval;

  if (box.isEmpty()) {
    // An empty computed box usually means the float32 camera-space
    // transform collapsed at large coordinates, not that the scene is
    // empty.  Keep current planes when they are usable instead of
    // resetting to a tiny default that would far-clip the scene; only
    // fall back to the default when the camera itself carries
    // degenerate values.
    const float curnear = camera->nearDistance.getValue();
    const float curfar = camera->farDistance.getValue();
    if (curnear == curnear && curfar == curfar  // not NaN
        && curfar > 0.0f && curfar > SbAbs(curnear)) {
      return;
    }
    nearval = 1;
    farval = 10;
  }
  else {
    // In camera space the camera looks down -Z, so geometry with
    // z > 0 is behind the camera.  Behind-camera content cannot be
    // seen, and letting it drag the near plane to large negative
    // values destroys the orthographic depth precision; in the
    // perspective branch below the nearlimit push would then also
    // land in front of all real content and clip everything away.
    // (A scene box that reaches behind the camera is normal: any UI
    // chrome attached at the world origin -- e.g. a workbench grid --
    // sits there while the camera frames geometry 1e9 mm away.)
    if (box.getMin()[2] >= 0.0f) {
      // The whole box is behind the camera: there is nothing to
      // frame. Keep the current planes instead of guessing.
      return;
    }

    // Clamp the near side to the in-front half-space.  nearest is the
    // depth of the nearest in-front content.  It is exact when the
    // whole box stays in front of the camera; when the box straddles
    // the camera (chrome behind, geometry in front) the box alone
    // cannot tell where the in-front content starts, so fall back to
    // the camera's current planes, which are known-good (they come
    // from fitToBox or a previous sane update).
    const float zmax = SbMin(box.getMax()[2], 0.0f);
    float nearest = -zmax;

    nearval = -zmax - clippingOffset;
    farval = -box.getMin()[2] + clippingOffset;

    if (nearest <= 0.0f) {
      const float curnear = camera->nearDistance.getValue();
      const float curfar = camera->farDistance.getValue();
      if (curnear > 0.0f) {
        nearest = curnear;
      }
      else if (curfar > 0.0f) {
        nearest = curfar * 1e-3f;
      }
    }

    // A single distant outlier on the view axis (an object at the
    // world origin seen from a camera framing geometry at 1e9 mm)
    // inflates farval by orders of magnitude.  The resulting
    // near/far span would collapse the depth resolution onto the
    // nearby content (visible as z-fighting), so cap the span at a
    // depth ratio the 24-bit depth buffer can still resolve.  Far
    // content beyond the cap is cleanly far-clipped; a later
    // view-all recomputes sane values once the camera has moved out.
    const float MAX_DEPTH_RATIO = 1000.0f;
    if (nearest > 0.0f) {
      const float maxfar = nearest * MAX_DEPTH_RATIO;
      if (farval > maxfar) {
        farval = maxfar;
      }
    }

    if (!camera->isOfType(SoOrthographicCamera::getClassTypeId()) && farval <= 0.0f) return;

    if (camera->isOfType(SoPerspectiveCamera::getClassTypeId())) {
      float nearlimit;
      if (this->autoclipping == SoRenderManager::FIXED_NEAR_PLANE) {
        nearlimit = this->nearplanevalue;
      } else {
        int depthbits = -1; // FIXME:   (20070628 frodo)
        if (depthbits < 0) depthbits = 32;
        int use_bits = (int) (float(depthbits) * (1.0f - this->nearplanevalue));
        float r = (float) pow(2.0, double(use_bits));
        nearlimit = farval / r;
      }

      if (nearlimit >= farval) {
        nearlimit = farval / 5000.0f;
      }

      // The depth-precision heuristic must never clip real content:
      // pushing the near plane past the nearest in-front geometry
      // makes objects disappear completely (e.g. farval inflated by
      // a distant outlier used to push near from ~15 mm to ~4e5 mm).
      if (nearest > 0.0f && nearlimit > nearest) {
        nearlimit = nearest;
      }

      if (nearval < nearlimit) {
        nearval = nearlimit;
      }
    }
  }

  const float SLACK = 0.001f;
  const float newnear = nearval >= 0 ? nearval * (1.0f - SLACK) : nearval * (1.0f + SLACK);
  const float newfar = farval >= 0 ? farval * (1.0f + SLACK) : farval * (1.0f - SLACK);

  const float neareps = nearval * SLACK * SLACK;
  const float fareps = farval * SLACK * SLACK;

  const float oldnear = camera->nearDistance.getValue();
  const float oldfar = camera->farDistance.getValue();

  // check that the values have changed before setting the fields to
  // avoid continuous redraws on static scenes. Use an epsilon value
  // when comparing

  if (SbAbs(oldnear - newnear) > SbAbs(neareps)) {
    camera->nearDistance = newnear;
  }
  if (SbAbs(oldfar - newfar) > SbAbs(fareps)) {
    camera->farDistance = newfar;
  }
}

void
SoRenderManagerP::getCameraCoordinateSystem(SbMatrix & matrix,
                                            SbMatrix & inverse)
{
  SoCamera * camera = this->camera;
  SoNode * scene = this->scene;
  assert(camera && scene);

  matrix = inverse = SbMatrix::identity();

  if (!this->searchaction) {
    this->searchaction = new SoSearchAction;
  }

  this->searchaction->reset();
  this->searchaction->setSearchingAll(TRUE);
  this->searchaction->setInterest(SoSearchAction::FIRST);
  this->searchaction->setNode(camera);
  this->searchaction->apply(scene);

  if (this->searchaction->getPath()) {
    if (!this->getmatrixaction) {
      this->getmatrixaction =
        new SoGetMatrixAction(this->glaction->getViewportRegion());
    } else {
      this->getmatrixaction->setViewportRegion(this->glaction->getViewportRegion());
    }
    this->getmatrixaction->apply(this->searchaction->getPath());
    matrix = this->getmatrixaction->getMatrix();
    inverse = this->getmatrixaction->getInverse();
  }
  this->searchaction->reset();
}

//**********************************************************************************
// Superimposition
//**********************************************************************************

class SuperimpositionP {
public:
  SoNode * scene;
  SbBool enabled;
  SoRenderManager * manager;
  SoNodeSensor * sensor;
  uint32_t stateflags;
  int transparencytype;
};

SoRenderManager::Superimposition::Superimposition(SoNode * scene,
                                                  SbBool enabled,
                                                  SoRenderManager * manager,
                                                  uint32_t flags)
{
  assert(scene != NULL);
  PRIVATE(this) = new SuperimpositionP;

  PRIVATE(this)->scene = scene;
  PRIVATE(this)->scene->ref();

  PRIVATE(this)->enabled = enabled;
  PRIVATE(this)->stateflags = flags;

  PRIVATE(this)->transparencytype = INHERIT_TRANSPARENCY_TYPE;

  PRIVATE(this)->manager = manager;
  PRIVATE(this)->sensor = new SoNodeSensor(Superimposition::changeCB, this);
  PRIVATE(this)->sensor->attach(PRIVATE(this)->scene);
}

SoRenderManager::Superimposition::~Superimposition()
{
  PRIVATE(this)->scene->unref();
  delete PRIVATE(this)->sensor;
  delete PRIVATE(this);
}

int
SoRenderManager::Superimposition::getStateFlags(void) const
{
  return PRIVATE(this)->stateflags;
}

void
SoRenderManager::Superimposition::render(SoGLRenderAction * action, SbBool clearcolorbuffer)
{
  if (!PRIVATE(this)->enabled) return;

  SoGLRenderAction::TransparencyType oldttype = action->getTransparencyType();
  if (PRIVATE(this)->transparencytype != INHERIT_TRANSPARENCY_TYPE) {
    action->setTransparencyType((SoGLRenderAction::TransparencyType) PRIVATE(this)->transparencytype);
  }
  SbBool zbufferwason = glIsEnabled(GL_DEPTH_TEST) ? TRUE : FALSE;

  PRIVATE(this)->stateflags & Superimposition::ZBUFFERON ?
    glEnable(GL_DEPTH_TEST):
    glDisable(GL_DEPTH_TEST);

  GLbitfield clearflags = clearcolorbuffer ? GL_COLOR_BUFFER_BIT : 0;
  if (PRIVATE(this)->stateflags & Superimposition::CLEARZBUFFER) {
    clearflags |= GL_DEPTH_BUFFER_BIT;
  }

  PRIVATE(this)->manager->renderScene(action, PRIVATE(this)->scene, (uint32_t) clearflags);

  zbufferwason ?
    glEnable(GL_DEPTH_TEST):
    glDisable(GL_DEPTH_TEST);

  if (PRIVATE(this)->transparencytype != INHERIT_TRANSPARENCY_TYPE) {
    action->setTransparencyType(oldttype);
  }
}

void
SoRenderManager::Superimposition::setEnabled(SbBool yes)
{
  PRIVATE(this)->enabled = yes;
}

void
SoRenderManager::Superimposition::changeCB(void * data, SoSensor * COIN_UNUSED_ARG(sensor))
{
  Superimposition * thisp = (Superimposition *) data;
  assert(thisp && PRIVATE(thisp)->manager);
  if (PRIVATE(thisp)->stateflags & Superimposition::AUTOREDRAW) {
    PRIVATE(thisp)->manager->scheduleRedraw();
  }
}

void
SoRenderManager::Superimposition::setTransparencyType(SoGLRenderAction::TransparencyType type)
{
  PRIVATE(this)->transparencytype = (int) type;
}

void
SoRenderManagerP::invokePreRenderCallbacks(void)
{
  std::vector<RenderCBTouple>::const_iterator cbit =
    this->preRenderCallbacks.begin();
  while (cbit != this->preRenderCallbacks.end()) {
    cbit->first(cbit->second, PUBLIC(this));
    ++cbit;
  }
}

void
SoRenderManagerP::invokePostRenderCallbacks(void)
{
  std::vector<RenderCBTouple>::const_iterator cbit =
    this->postRenderCallbacks.begin();
  while (cbit != this->postRenderCallbacks.end()) {
    cbit->first(cbit->second, PUBLIC(this));
    ++cbit;
  }
}

#undef INHERIT_TRANSPARENCY_TYPE
#undef PRIVATE
#undef PUBLIC

void
SoRenderManagerRootSensor::notify(SoNotList * l)
{
  l->print();
  (void)fprintf(stdout, "end\n");

  inherited::notify(l);
}

SbBool
SoRenderManagerRootSensor::debug(void)
{
  if (SoRenderManagerRootSensor::debugrootnotifications == -1) {
    const char * env = coin_getenv("COIN_DEBUG_ROOT_NOTIFICATIONS");
    SoRenderManagerRootSensor::debugrootnotifications = env && (atoi(env) > 0);
  }
  return SoRenderManagerRootSensor::debugrootnotifications ? TRUE : FALSE;
}
