#ifndef COIN_SOMODELMATRIXELEMENT_H
#define COIN_SOMODELMATRIXELEMENT_H

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

#include <Inventor/elements/SoAccumulatedElement.h>
#include <Inventor/SbDPMatrix.h>
#include <Inventor/SbMatrix.h>
#include <Inventor/lists/SbPList.h>

class COIN_DLL_API SoModelMatrixElement : public SoAccumulatedElement {
  typedef SoAccumulatedElement inherited;

  SO_ELEMENT_HEADER(SoModelMatrixElement);
public:
  static void initClass(void);
protected:
  virtual ~SoModelMatrixElement();

public:
  void init(SoState * state) override;
  void push(SoState * state) override;
  SbBool matches(const SoElement * element) const override;
  static void makeIdentity(SoState * const state, SoNode * const node);
  static void set(SoState * const state, SoNode * const node,
                  const SbMatrix & matrix);
  static void setCullMatrix(SoState * state, SoNode * node,
                            const SbMatrix & matrix);
  static void mult(SoState * const state, SoNode * const node,
                   const SbMatrix & matrix);
  static void translateBy(SoState * const state, SoNode * const node,
                          const SbVec3f & translation);
  static void rotateBy(SoState * const state, SoNode * const node,
                       const SbRotation & rotation);
  static void scaleBy(SoState * const state, SoNode * const node,
                      const SbVec3f & scaleFactor);

  static SbMatrix pushMatrix(SoState * const state);
  static void popMatrix(SoState * const state, const SbMatrix & matrix);

  static const SbMatrix & getCombinedCullMatrix(SoState * const state);
  static const SbMatrix & get(SoState * const state);
  static const SbMatrix & get(SoState * const state, SbBool & isIdentity);
  // Double precision shadow of the model matrix (see modelMatrixDP).
  // View frustum culling uses this to keep the corner transform exact
  // for absolute coordinates beyond the float32 ULP.
  static const SbDPMatrix & getDPMatrix(SoState * const state);

  const SbMatrix & getModelMatrix(void) const;

protected:
  virtual void makeEltIdentity(void);
  virtual void setElt(const SbMatrix & matrix);
  virtual void multElt(const SbMatrix & matrix);
  virtual void translateEltBy(const SbVec3f & translation);
  virtual void rotateEltBy(const SbRotation & rotation);
  virtual void scaleEltBy(const SbVec3f & scaleFactor);
  virtual SbMatrix pushMatrixElt(void);
  virtual void popMatrixElt(const SbMatrix & matrix);

protected:
  SbMatrix modelMatrix;
  // Double precision shadow of modelMatrix, updated with the same
  // operations in the same order. The float32 modelMatrix rounds to the
  // float32 ULP of absolute coordinates (64 mm at 1e9 mm), which destroys
  // viewport-sized local offsets once a large absolute anchor translation
  // has been accumulated; the double shadow keeps them exact for the
  // double precision GL download (SoGLViewingMatrixElement::loadFullMatrix).
  // Consumers of get()/getCombinedCullMatrix() keep the float32 value and
  // therefore see unchanged (legacy) behaviour.
  SbDPMatrix modelMatrixDP;
  SbMatrix cullMatrix;
  SbMatrix combinedMatrix;
  uint32_t flags;

};

#endif // !COIN_SOMODELMATRIXELEMENT_H
