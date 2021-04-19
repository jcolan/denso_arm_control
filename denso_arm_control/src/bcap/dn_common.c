/**
 * Software License Agreement (MIT License)
 *
 * @copyright Copyright (c) 2015 DENSO WAVE INCORPORATED
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "stdint.h"
#include <stdlib.h>
#include <string.h>

#define _USE_LINUX_API

#if defined(_USE_WIN_API)
#include <windows.h>
#elif defined(_USE_LINUX_API)
#include <errno.h>
#ifndef _wcsicmp
#define _wcsicmp wcscasecmp
#endif
#ifndef __USE_XOPEN
#define __USE_XOPEN
#endif
#endif

#include "denso_communication/dn_common.h"

/**
 * @def   _LEN_VARIANT2BSTR
 * @brief A definition for the string length for converting VARIANT to BSTR.
 */
#define _LEN_VARIANT2BSTR (100)

#ifndef _OLEAUTO_H_

/**
 * @fn        BSTR SysAllocString(const wchar_t *sz)
 * @brief     Allocates and returns BSTR.
 * @param[in] sz Unicode string to be copied. This must be NULL terminated.
 * @note      If there is no enough memory space, then return NULL.
 */
BSTR
SysAllocString(const wchar_t *sz)
{
  if (sz == NULL)
    return NULL;

  return SysAllocStringLen(sz, wcslen(sz));
}

/**
 * @fn        BSTR SysAllocStringLen(const wchar_t *pch, uint16_t cch)
 * @brief     Allocates and returns BSTR.
 * @param[in] pch Unicode string to be copied. This must be NULL terminated.
 * @param[in] cch The number of characters to be copied.
 * @note      If there is no enough memory space, then return NULL.
 */
BSTR
SysAllocStringLen(const wchar_t *pch, uint16_t cch)
{
  uint16_t minlen;
  BSTR bstr = NULL;

  minlen = sizeof(wchar_t) * (cch + 1);
  bstr = (BSTR) malloc(minlen);

  if (bstr != NULL) {
    memset(bstr, 0, minlen);

    if (pch != NULL) {
      minlen = wcslen(pch);
      minlen = (cch < minlen ? cch : minlen);
      memcpy(bstr, pch, sizeof(wchar_t) * minlen);
    }
  }

  return bstr;
}

/**
 * @fn            void SysFreeString(BSTR bstr)
 * @brief         Releases the memory of BSTR.
 * @param[in,out] bstr Unicode string to be freed.
 */
void
SysFreeString(BSTR bstr)
{
  if (bstr != NULL) {
    free(bstr);
    bstr = NULL;
  }
}

/**
 * @fn        uint16 SysStringLen(BSTR bstr)
 * @brief     Gets and returns the number of characters of BSTR.
 * @param[in] bstr Unicode string to be gotten. This must be NULL terminated.
 */
uint16_t
SysStringLen(BSTR bstr)
{
  uint16_t len = 0;

  if (bstr != NULL) {
    len = wcslen(bstr);
  }

  return len;
}

/**
 * @fn        SAFEARRAY* SafeArrayCreateVector(uint16_t vt, int32_t lLbound, uint32_t cElements)
 * @brief     Allocates and returns SAFEARRAY.
 * @param[in] vt Variant type.
 * @param[in] lLbound The lower bound of array. This should be 0.
 * @param[in] cElements The number of elements.
 * @note      If there is no enough memory space, then return NULL.
 */
SAFEARRAY*
SafeArrayCreateVector(uint16_t vt, int32_t lLbound, uint32_t cElements)
{
  int sz;
  SAFEARRAY* psa = NULL;

  psa = (SAFEARRAY*) malloc(sizeof(SAFEARRAY));

  if (psa != NULL) {
    memset(psa, 0, sizeof(SAFEARRAY));

    psa->cDims = 1;
    psa->vt = vt;
    psa->rgsabound[0].lLbound = lLbound;
    psa->rgsabound[0].cElements = cElements;

    if (cElements > 0) {
      switch (vt) {
        case VT_UI1:
          sz = 1;
          break;
        case VT_I2:
        case VT_UI2:
        case VT_BOOL:
          sz = 2;
          break;
        case VT_I4:
        case VT_UI4:
        case VT_R4:
          sz = 4;
          break;
        case VT_R8:
        case VT_CY:
          sz = 8;
          break;
        case VT_DATE:
          sz = sizeof(DATE);
          break;
        case VT_BSTR:
          sz = sizeof(BSTR);
          break;
        case VT_VARIANT:
          sz = sizeof(VARIANT);
          break;
        default:
          free(psa);
          psa = NULL;
          goto exit_proc;
      }

      psa->cbElements = sz;
      psa->pvData = malloc(cElements * sz);

      if (psa->pvData == NULL) {
        free(psa);
        psa = NULL;
        goto exit_proc;
      }

      memset(psa->pvData, 0, cElements * sz);
    }
  }

exit_proc:
  return psa;
}

/**
 * @fn            HRESULT SafeArrayDestroy(SAFEARRAY *psa)
 * @brief         Releases the memory of SAFEARRAY.
 * @param[in,out] psa SAFEARRAY to be freed.
 */
HRESULT
SafeArrayDestroy(SAFEARRAY *psa)
{
  int32_t i;

  if (psa != NULL) {
    if (psa->pvData != NULL) {
      switch (psa->vt) {
        case VT_BSTR:
          for (i = 0; i < psa->rgsabound[0].cElements; i++) {
            SysFreeString(*((BSTR*) psa->pvData + i));
          }
          break;
        case VT_VARIANT:
          for (i = 0; i < psa->rgsabound[0].cElements; i++) {
            VariantClear(((VARIANT*) psa->pvData + i));
          }
          break;
        default:
          free(psa->pvData);
          break;
      }
      psa->pvData = NULL;
    }
    free(psa);
    psa = NULL;
  }

  return S_OK;
}

/**
 * @fn        uint16_t SafeArrayGetDim(SAFEARRAY *psa)
 * @brief     Gets and returns the dimension of SAFEARRAY.
 * @param[in] psa SAFEARRAY to be gotten.
 */
uint16_t
SafeArrayGetDim(SAFEARRAY *psa)
{
  if (psa == NULL)
    return 0;

  return psa->cDims;
}

/**
 * @fn        uint32_t SafeArrayGetElemsize(SAFEARRAY *psa)
 * @brief     Gets and returns the size of an element.
 * @param[in] psa SAFEARRAY to be gotten.
 */
uint32_t
SafeArrayGetElemsize(SAFEARRAY *psa)
{
  if (psa == NULL)
    return 0;

  return psa->cbElements;
}

/**
 * @fn         HRESULT SafeArrayGetLBound(SAFEARRAY *psa, uint16_t nDim, int32_t *plLbound)
 * @brief      Gets the lower bound of SAFEARRAY.
 * @param[in]  psa SAFEARRAY to be gotten.
 * @param[in]  nDim The target dimension.
 * @param[out] plLbound The gotten lower bound.
 */
HRESULT
SafeArrayGetLBound(SAFEARRAY *psa, uint16_t nDim, int32_t *plLbound)
{
  if (psa == NULL || plLbound == NULL)
    return E_INVALIDARG;
  if (nDim <= 0 || psa->cDims < nDim)
    return DISP_E_BADINDEX;

  *plLbound = psa->rgsabound[nDim - 1].lLbound;

  return S_OK;
}

/**
 * @fn         HRESULT SafeArrayGetUBound(SAFEARRAY *psa, uint16_t nDim, int32_t *plUbound)
 * @brief      Gets the upper bound of SAFEARRAY.
 * @param[in]  psa SAFEARRAY to be gotten.
 * @param[in]  nDim The target dimension.
 * @param[out] plUbound The gotten upper bound.
 */
HRESULT
SafeArrayGetUBound(SAFEARRAY *psa, uint16_t nDim, int32_t *plUbound)
{
  if (psa == NULL || plUbound == NULL)
    return E_INVALIDARG;
  if (nDim <= 0 || psa->cDims < nDim)
    return DISP_E_BADINDEX;

  *plUbound = (psa->rgsabound[nDim - 1].cElements
      + psa->rgsabound[nDim - 1].lLbound - 1);

  return S_OK;
}

/**
 * @fn         HRESULT SafeArrayGetVartype(SAFEARRAY *psa, uint16_t *pvt)
 * @brief      Gets the variant type of SAFEARRAY.
 * @param[in]  psa SAFEARRAY to be gotten.
 * @param[out] pvt The gotten variant type.
 */
HRESULT
SafeArrayGetVartype(SAFEARRAY *psa, uint16_t *pvt)
{
  if (psa == NULL || pvt == NULL)
    return E_INVALIDARG;

  *pvt = psa->vt;

  return S_OK;
}

/**
 * @fn         HRESULT SafeArrayAccessData(SAFEARRAY *psa, void **ppvData)
 * @brief      Accesses the SAFEARRAY and gets the pointer of array data.
 * @param[in]  psa SAFEARRAY to be accessed.
 * @param[out] ppvData The gotten pointer.
 * @note  This function must be called before accessing SAFEARRAY data.
 */
HRESULT
SafeArrayAccessData(SAFEARRAY *psa, void **ppvData)
{
  if (psa == NULL || ppvData == NULL)
    return E_INVALIDARG;

  *ppvData = psa->pvData;

  return S_OK;
}

/**
 * @fn        HRESULT SafeArrayUnaccessData(SAFEARRAY *psa)
 * @brief     Unaccesses the SAFEARRAY.
 * @param[in] psa SAFEARRAY to be unaccessed.
 * @note      This function must be called after accessing SAFEARRAY data.
 */
HRESULT
SafeArrayUnaccessData(SAFEARRAY *psa)
{
  if (psa == NULL)
    return E_INVALIDARG;

  return S_OK;
}

/**
 * @fn            void VariantInit(VARIANT *pvarg)
 * @brief         Initializes the VARIANT.
 * @param[in,out] pvarg VARIANT to be initialized.
 * @note          This function must be called before accessing VARIANT.
 */
void
VariantInit(VARIANT *pvarg)
{
  if (pvarg != NULL) {
    memset(pvarg, 0, sizeof(VARIANT));
  }
}

/**
 * @fn            void VariantClear(VARIANT *pvarg)
 * @brief         Clears the VARIANT.
 * @param[in,out] pvarg VARIANT to be cleared.
 * @note          This function must be called before destructing VARIANT.
 */
void
VariantClear(VARIANT *pvarg)
{
  if (pvarg != NULL) {
    if (pvarg->vt & VT_ARRAY) {
      if (pvarg->parray != NULL) {
        SafeArrayDestroy(pvarg->parray);
        pvarg->parray = NULL;
      }
    }
    else if (pvarg->vt == VT_BSTR) {
      if (pvarg->bstrVal != NULL) {
        SysFreeString(pvarg->bstrVal);
        pvarg->bstrVal = NULL;
      }
    }
    memset(pvarg, 0, sizeof(VARIANT));
  }
}

#if (_DN_USE_VARIANT_API)
/**
 * @fn         HRESULT VariantCopy(VARIANT *pvargDest, const VARIANT *pvargSrc)
 * @brief      Copies the source variant to destination variant.
 * @param[out] pvargDest The destination variant to be changed.
 * @param[in]  pvarSrc The source variant.
 */
HRESULT
VariantCopy(VARIANT *pvargDest, const VARIANT *pvargSrc)
{
  if ((pvargDest == NULL) || (pvargSrc == NULL)) {
    return E_INVALIDARG;
  }

  if (pvargDest == pvargSrc) {
    return S_OK;
  }

  VariantClear(pvargDest);

  if (pvargSrc->vt & VT_ARRAY) {
    int32_t i, lLbound = 0;
    uint32_t cbElements = 0, cElements;

    lLbound = pvargSrc->parray->rgsabound[0].lLbound;
    cElements = pvargSrc->parray->rgsabound[0].cElements;
    cbElements = pvargSrc->parray->cbElements;

    switch (pvargSrc->vt ^ VT_ARRAY) {
      case VT_I2:
      case VT_I4:
      case VT_R4:
      case VT_R8:
      case VT_CY:
      case VT_DATE:
      case VT_BOOL:
      case VT_UI1:
      case VT_UI2:
      case VT_UI4:
        pvargDest->vt = pvargSrc->vt;
        pvargDest->parray = SafeArrayCreateVector(pvargSrc->vt ^ VT_ARRAY,
            lLbound, cElements);
        memcpy(pvargDest->parray->pvData, pvargSrc->parray->pvData,
            cbElements * cElements);
        break;
      case VT_BSTR:
        pvargDest->vt = pvargSrc->vt;
        pvargDest->parray = SafeArrayCreateVector(VT_BSTR, lLbound, cElements);
        for (i = 0; i < cElements; i++) {
          *((BSTR*) pvargDest->parray->pvData + i) = SysAllocString(
              *((BSTR*) pvargSrc->parray->pvData + i));
        }
        break;
      case VT_VARIANT:
        pvargDest->vt = pvargSrc->vt;
        pvargDest->parray = SafeArrayCreateVector(VT_VARIANT, lLbound,
            cElements);
        for (i = 0; i < cElements; i++) {
          VariantCopy(((VARIANT*) pvargDest->parray->pvData + i),
              ((VARIANT*) pvargSrc->parray->pvData + i));
        }
        break;
      default:
        return DISP_E_BADVARTYPE;
    }
  } else {
    switch (pvargSrc->vt) {
      case VT_EMPTY:
      case VT_NULL:
      case VT_I2:
      case VT_I4:
      case VT_R4:
      case VT_R8:
      case VT_CY:
      case VT_DATE:
      case VT_BOOL:
      case VT_UI1:
      case VT_UI2:
      case VT_UI4:
        memcpy(pvargDest, pvargSrc, sizeof(VARIANT));
        break;
      case VT_BSTR:
        pvargDest->vt = VT_BSTR;
        pvargDest->bstrVal = SysAllocString(pvargSrc->bstrVal);
        break;
      default:
        return DISP_E_BADVARTYPE;
    }
  }

  return S_OK;
}

/**
 * @fn         HRESULT Variant2Bstr(BSTR *pbstr, VARIANT *pvarg)
 * @brief      Change VARIANT to BSTR.
 * @param[out] pbstr The destination BSTR.
 * @param[in]  pvarg The source VARIANT.
 */
static HRESULT
Variant2Bstr(BSTR *pbstr, VARIANT *pvarg)
{
  HRESULT hr = S_OK;
  char chStr[_LEN_VARIANT2BSTR + 1];
  wchar_t wchStr[_LEN_VARIANT2BSTR + 1];
  struct tm *tmVal;

  switch (pvarg->vt) {
    case VT_I2:
      swprintf(wchStr, _LEN_VARIANT2BSTR, L"%d", pvarg->iVal);
      break;
    case VT_I4:
      swprintf(wchStr, _LEN_VARIANT2BSTR, L"%d", pvarg->lVal);
      break;
    case VT_R4:
      swprintf(wchStr, _LEN_VARIANT2BSTR, L"%f", pvarg->fltVal);
      break;
    case VT_R8:
      swprintf(wchStr, _LEN_VARIANT2BSTR, L"%f", pvarg->dblVal);
      break;
    case VT_CY:
      swprintf(wchStr, _LEN_VARIANT2BSTR, L"%lld", pvarg->cyVal.int64);
      break;
    case VT_DATE:
      tmVal = gmtime(&pvarg->date);
      strftime(chStr, _LEN_VARIANT2BSTR, FORMAT_DATE2BSTR, tmVal);
      mbstowcs(wchStr, chStr, _LEN_VARIANT2BSTR);
      break;
    case VT_BOOL:
      swprintf(wchStr, _LEN_VARIANT2BSTR, L"%d", pvarg->boolVal);
      break;
    case VT_UI1:
      swprintf(wchStr, _LEN_VARIANT2BSTR, L"%u", pvarg->bVal);
      break;
    case VT_UI2:
      swprintf(wchStr, _LEN_VARIANT2BSTR, L"%u", pvarg->uiVal);
      break;
    case VT_UI4:
      swprintf(wchStr, _LEN_VARIANT2BSTR, L"%u", pvarg->ulVal);
      break;
  }

  *pbstr = SysAllocString(wchStr);

  return hr;
}

/**
 * @fn         HRESULT Bstr2Variant(VARIANT *pvarg, int16_t vt, BSTR bstr)
 * @brief      Change BSTR to VARIANT.
 * @param[out] pvarg The destination VARIANT.
 * @param[in]  bstr The source BSTR.
 */
static HRESULT
Bstr2Variant(VARIANT *pvarg, int16_t vt, BSTR bstr)
{
  HRESULT hr = S_OK;
  int flag = 0;
  int16_t sz_bstr;
  char *chRet, *chStr = NULL;
  wchar_t *wchRet;
  struct tm tmVal;
  VARIANT vntTmp;

  VariantInit(&vntTmp);

  if (pvarg->bstrVal == bstr) {
    flag = 1;
  }

  switch (vt) {
    case VT_I2:
    case VT_I4:
    case VT_BOOL:
      errno = 0;
      vntTmp.lVal = (int32_t) wcstol(bstr, &wchRet, 0);
      if (wchRet == bstr) {
        hr = DISP_E_TYPEMISMATCH;
      }
      else if (errno == ERANGE) {
        hr = DISP_E_OVERFLOW;
      }
      else if ((vt == VT_I2) || (vt == VT_BOOL)) {
        vntTmp.vt = VT_I4;
        hr = VariantChangeType(&vntTmp, &vntTmp, 0, vt);
      }
      break;
    case VT_R4:
      errno = 0;
      vntTmp.fltVal = wcstof(bstr, &wchRet);
      if (wchRet == bstr) {
        hr = DISP_E_TYPEMISMATCH;
      }
      else if (errno == ERANGE) {
        hr = DISP_E_OVERFLOW;
      }
      break;
    case VT_R8:
      errno = 0;
      vntTmp.dblVal = wcstod(bstr, &wchRet);
      if (wchRet == bstr) {
        hr = DISP_E_TYPEMISMATCH;
      }
      else if (errno == ERANGE) {
        hr = DISP_E_OVERFLOW;
      }
      break;
    case VT_CY:
      errno = 0;
      vntTmp.cyVal.int64 = (int64_t) wcstoll(bstr, &wchRet, 0);
      if (wchRet == bstr) {
        hr = DISP_E_TYPEMISMATCH;
      }
      else if (errno == ERANGE) {
        hr = DISP_E_OVERFLOW;
      }
      break;
    case VT_UI1:
    case VT_UI2:
    case VT_UI4:
      errno = 0;
      vntTmp.ulVal = (uint32_t) wcstoul(bstr, &wchRet, 0);
      if (wchRet == bstr) {
        hr = DISP_E_TYPEMISMATCH;
      }
      else if (errno == ERANGE) {
        hr = DISP_E_OVERFLOW;
      }
      else if ((vt == VT_UI1) || (vt == VT_UI2)) {
        vntTmp.vt = VT_UI4;
        hr = VariantChangeType(&vntTmp, &vntTmp, 0, vt);
      }
      else if (*bstr == L'-') {
        hr = DISP_E_OVERFLOW;
      }
      break;
    case VT_DATE:
      sz_bstr = SysStringLen(bstr);
      chStr = (char *) malloc(sz_bstr + 1);
      if (chStr == NULL) {
        hr = E_OUTOFMEMORY;
        break;
      }
      wcstombs(chStr, bstr, sz_bstr);
      memset(&tmVal, 0, sizeof(struct tm));
      chRet = strptime(chStr, FORMAT_DATE2BSTR, &tmVal);
      if (chRet == NULL) {
        hr = DISP_E_TYPEMISMATCH;
      } else {
        vntTmp.date = mktime(&tmVal);
      }
      free(chStr);
      break;
  }

  if (SUCCEEDED(hr)) {
    *pvarg = vntTmp;
    if (flag) {
      SysFreeString(bstr);
    }
  }

  VariantClear(&vntTmp);

  return hr;
}

/**
 * @fn         HRESULT VariantChangeType(VARIANT *pvargDest, VARIANT *pvarSrc, uint16_t wFlags, uint16_t vt)
 * @brief      Changes the source variant to destination variant with the indicated type.
 * @param[out] pvargDest The destination variant to be changed.
 * @param[in]  pvarSrc The source variant.
 * @param[in]  wFlags Flags.
 * @param[in]  vt The variant type.
 * @note       This function is not sufficiently compatible with Windows.
 */
HRESULT
VariantChangeType(VARIANT *pvargDest, VARIANT *pvarSrc, uint16_t wFlags,
    uint16_t vt)
{
  HRESULT hr = S_OK;

  if ((pvargDest == NULL) || (pvarSrc == NULL)) {
    return E_INVALIDARG;
  }

  switch (vt) {
    case VT_EMPTY:
    case VT_NULL:
    case VT_I2:
    case VT_I4:
    case VT_R4:
    case VT_R8:
    case VT_CY:
    case VT_DATE:
    case VT_BSTR:
    case VT_BOOL:
    case VT_UI1:
    case VT_UI2:
    case VT_UI4:
      break;
    default:
      return DISP_E_BADVARTYPE;
  }

  if (pvargDest != pvarSrc) {
    VariantClear(pvargDest);
    if (vt == pvarSrc->vt) {
      return VariantCopy(pvargDest, pvarSrc);
    }
  } else {
    if (vt == pvarSrc->vt) {
      return S_OK;
    }
  }

  switch (pvarSrc->vt) {
    case VT_EMPTY:
    case VT_NULL:
      memset(pvargDest, 0, sizeof(VARIANT));
      break;
    case VT_I2:
      switch (vt) {
        case VT_I4:
          pvargDest->lVal = pvarSrc->iVal;
          break;
        case VT_R4:
          pvargDest->fltVal = (float) pvarSrc->iVal;
          break;
        case VT_R8:
          pvargDest->dblVal = (double) pvarSrc->iVal;
          break;
        case VT_CY:
          pvargDest->cyVal.int64 = pvarSrc->iVal;
          break;
        case VT_DATE:
          pvargDest->date = (DATE) pvarSrc->iVal;
          break;
        case VT_BSTR:
          hr = Variant2Bstr(&pvargDest->bstrVal, pvarSrc);
          break;
        case VT_BOOL:
          pvargDest->boolVal = (pvarSrc->iVal ? VARIANT_TRUE : VARIANT_FALSE);
          break;
        case VT_UI1:
          if ((pvarSrc->iVal < 0) || (256 <= pvarSrc->iVal)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->bVal = (uint8_t) pvarSrc->iVal;
          break;
        case VT_UI2:
          pvargDest->uiVal = (uint16_t) pvarSrc->iVal;
          break;
        case VT_UI4:
          pvargDest->ulVal = (uint32_t) pvarSrc->iVal;
          break;
      }
      break;
    case VT_I4:
      switch (vt) {
        case VT_I2:
          if ((pvarSrc->lVal < -32768) || (32768 <= pvarSrc->lVal)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->iVal = (int16_t) pvarSrc->lVal;
          break;
        case VT_R4:
          pvargDest->fltVal = (float) pvarSrc->lVal;
          break;
        case VT_R8:
          pvargDest->dblVal = (double) pvarSrc->lVal;
          break;
        case VT_CY:
          pvargDest->cyVal.int64 = pvarSrc->lVal;
          break;
        case VT_DATE:
          pvargDest->date = (DATE) pvarSrc->lVal;
          break;
        case VT_BSTR:
          hr = Variant2Bstr(&pvargDest->bstrVal, pvarSrc);
          break;
        case VT_BOOL:
          pvargDest->boolVal = (pvarSrc->lVal ? VARIANT_TRUE : VARIANT_FALSE);
          break;
        case VT_UI1:
          if ((pvarSrc->lVal < 0) || (256 <= pvarSrc->lVal)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->bVal = (uint8_t) pvarSrc->lVal;
          break;
        case VT_UI2:
          if ((pvarSrc->lVal < 0) || (65536 <= pvarSrc->lVal)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->uiVal = (uint16_t) pvarSrc->lVal;
          break;
        case VT_UI4:
          pvargDest->ulVal = (uint32_t) pvarSrc->lVal;
          break;
      }
      break;
    case VT_R4:
      switch (vt) {
        case VT_I2:
          if ((pvarSrc->fltVal < -32768.0f) || (32768.0f <= pvarSrc->fltVal)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->iVal = (int16_t) pvarSrc->fltVal;
          break;
        case VT_I4:
          if ((pvarSrc->fltVal < -2147483648.0f)
              || (2147483648.0f <= pvarSrc->fltVal)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->lVal = (int32_t) pvarSrc->fltVal;
          break;
        case VT_R8:
          pvargDest->dblVal = (double) pvarSrc->fltVal;
          break;
        case VT_CY:
          if ((pvarSrc->fltVal < -9223372036854775808.0f)
              || (9223372036854775808.0f <= pvarSrc->fltVal)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->cyVal.int64 = (int64_t) pvarSrc->fltVal;
          break;
        case VT_DATE:
          pvargDest->date = (DATE) pvarSrc->fltVal;
          break;
        case VT_BSTR:
          hr = Variant2Bstr(&pvargDest->bstrVal, pvarSrc);
          break;
        case VT_BOOL:
          pvargDest->boolVal = (pvarSrc->fltVal ? VARIANT_TRUE : VARIANT_FALSE);
          break;
        case VT_UI1:
          if ((pvarSrc->fltVal < 0.0f) || (256.0f <= pvarSrc->fltVal)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->bVal = (uint8_t) pvarSrc->fltVal;
          break;
        case VT_UI2:
          if ((pvarSrc->fltVal < 0.0f) || (65536.0f <= pvarSrc->fltVal)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->uiVal = (uint16_t) pvarSrc->fltVal;
          break;
        case VT_UI4:
          if ((pvarSrc->fltVal < 0.0f) || (4294967296.0f <= pvarSrc->fltVal)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->ulVal = (uint32_t) pvarSrc->fltVal;
          break;
      }
      break;
    case VT_R8:
      switch (vt) {
        case VT_I2:
          if ((pvarSrc->dblVal < -32768.0) || (32768.0 <= pvarSrc->dblVal)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->iVal = (int16_t) pvarSrc->dblVal;
          break;
        case VT_I4:
          if ((pvarSrc->dblVal < -2147483648.0)
              || (2147483648.0 <= pvarSrc->dblVal)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->lVal = (int32_t) pvarSrc->dblVal;
          break;
        case VT_R4:
          pvargDest->fltVal = (float) pvarSrc->dblVal;
          break;
        case VT_CY:
          if ((pvarSrc->dblVal < -9223372036854775808.0)
              || (9223372036854775808.0 <= pvarSrc->dblVal)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->cyVal.int64 = (int64_t) pvarSrc->dblVal;
          break;
        case VT_DATE:
          pvargDest->date = (DATE) pvarSrc->dblVal;
          break;
        case VT_BSTR:
          hr = Variant2Bstr(&pvargDest->bstrVal, pvarSrc);
          break;
        case VT_BOOL:
          pvargDest->boolVal = (pvarSrc->dblVal ? VARIANT_TRUE : VARIANT_FALSE);
          break;
        case VT_UI1:
          if ((pvarSrc->dblVal < 0.0f) || (256.0f <= pvarSrc->dblVal)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->bVal = (uint8_t) pvarSrc->dblVal;
          break;
        case VT_UI2:
          if ((pvarSrc->dblVal < 0.0f) || (65536.0f <= pvarSrc->dblVal)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->uiVal = (uint16_t) pvarSrc->dblVal;
          break;
        case VT_UI4:
          if ((pvarSrc->dblVal < 0.0f) || (4294967296.0f <= pvarSrc->dblVal)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->ulVal = (uint32_t) pvarSrc->dblVal;
          break;
      }
      break;
    case VT_CY:
      switch (vt) {
        case VT_I2:
          if ((pvarSrc->cyVal.int64 < -32768) || (32768 <= pvarSrc->cyVal.int64)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->iVal = (int16_t) pvarSrc->cyVal.int64;
          break;
        case VT_I4:
          if ((pvarSrc->cyVal.int64 < -2147483648LL)
              || (2147483648LL <= pvarSrc->cyVal.int64)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->lVal = (int32_t) pvarSrc->cyVal.int64;
          break;
        case VT_R4:
          pvargDest->fltVal = (float) pvarSrc->cyVal.int64;
          break;
        case VT_R8:
          pvargDest->dblVal = (double) pvarSrc->cyVal.int64;
          break;
        case VT_DATE:
          pvargDest->date = (DATE) pvarSrc->cyVal.int64;
          break;
        case VT_BSTR:
          hr = Variant2Bstr(&pvargDest->bstrVal, pvarSrc);
          break;
        case VT_BOOL:
          pvargDest->boolVal =
              (pvarSrc->cyVal.int64 ? VARIANT_TRUE : VARIANT_FALSE);
          break;
        case VT_UI1:
          if ((pvarSrc->cyVal.int64 < 0) || (256 <= pvarSrc->cyVal.int64)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->bVal = (uint8_t) pvarSrc->cyVal.int64;
          break;
        case VT_UI2:
          if ((pvarSrc->cyVal.int64 < 0) || (65536 <= pvarSrc->cyVal.int64)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->uiVal = (uint16_t) pvarSrc->cyVal.int64;
          break;
        case VT_UI4:
          if ((pvarSrc->cyVal.int64 < 0) || (4294967296LL <= pvarSrc->cyVal.int64)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->ulVal = (uint32_t) pvarSrc->cyVal.int64;
          break;
      }
      break;
    case VT_DATE:
      switch (vt) {
        case VT_I2:
          if ((pvarSrc->date < -32768) || (32768 <= pvarSrc->date)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->iVal = (int16_t) pvarSrc->date;
          break;
        case VT_I4:
          if ((pvarSrc->date < -2147483648LL) || (2147483648LL <= pvarSrc->date)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->lVal = (int32_t) pvarSrc->date;
          break;
        case VT_R4:
          pvargDest->fltVal = (float) pvarSrc->date;
          break;
        case VT_R8:
          pvargDest->dblVal = (double) pvarSrc->date;
          break;
        case VT_CY:
          pvargDest->cyVal.int64 = (int64_t) pvarSrc->date;
          break;
        case VT_BSTR:
          hr = Variant2Bstr(&pvargDest->bstrVal, pvarSrc);
          break;
        case VT_BOOL:
          pvargDest->boolVal = (pvarSrc->date ? VARIANT_TRUE : VARIANT_FALSE);
          break;
        case VT_UI1:
          if ((pvarSrc->date < 0) || (256 <= pvarSrc->date)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->bVal = (uint8_t) pvarSrc->date;
          break;
        case VT_UI2:
          if ((pvarSrc->date < 0) || (65536 <= pvarSrc->date)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->uiVal = (uint16_t) pvarSrc->date;
          break;
        case VT_UI4:
          if ((pvarSrc->date < 0) || (4294967296LL <= pvarSrc->date)) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->ulVal = (uint32_t) pvarSrc->date;
          break;
      }
      break;
    case VT_BSTR:
      hr = Bstr2Variant(pvargDest, vt, pvarSrc->bstrVal);
      break;
    case VT_BOOL:
      switch (vt) {
        case VT_I2:
          pvargDest->iVal = pvarSrc->boolVal;
          break;
        case VT_I4:
          pvargDest->lVal = pvarSrc->boolVal;
          break;
        case VT_R4:
          pvargDest->fltVal = (float) pvarSrc->boolVal;
          break;
        case VT_R8:
          pvargDest->dblVal = (double) pvarSrc->boolVal;
          break;
        case VT_CY:
          pvargDest->cyVal.int64 = pvarSrc->boolVal;
          break;
        case VT_DATE:
          pvargDest->date = (DATE) pvarSrc->boolVal;
          break;
        case VT_BSTR:
          hr = Variant2Bstr(&pvargDest->bstrVal, pvarSrc);
          break;
        case VT_UI1:
          pvargDest->bVal = (uint8_t) pvarSrc->boolVal;
          break;
        case VT_UI2:
          pvargDest->uiVal = (uint16_t) pvarSrc->boolVal;
          break;
        case VT_UI4:
          pvargDest->ulVal = (uint32_t) pvarSrc->boolVal;
          break;
      }
      break;
    case VT_UI1:
      switch (vt) {
        case VT_I2:
          pvargDest->iVal = (int16_t) pvarSrc->bVal;
          break;
        case VT_I4:
          pvargDest->lVal = (int32_t) pvarSrc->bVal;
          break;
        case VT_R4:
          pvargDest->fltVal = (float) pvarSrc->bVal;
          break;
        case VT_R8:
          pvargDest->dblVal = (double) pvarSrc->bVal;
          break;
        case VT_CY:
          pvargDest->cyVal.int64 = (int64_t) pvarSrc->bVal;
          break;
        case VT_DATE:
          pvargDest->date = (DATE) pvarSrc->bVal;
          break;
        case VT_BSTR:
          hr = Variant2Bstr(&pvargDest->bstrVal, pvarSrc);
          break;
        case VT_BOOL:
          pvargDest->boolVal = (pvarSrc->bVal ? VARIANT_TRUE : VARIANT_FALSE);
          break;
        case VT_UI2:
          pvargDest->uiVal = pvarSrc->bVal;
          break;
        case VT_UI4:
          pvargDest->ulVal = pvarSrc->bVal;
          break;
      }
      break;
    case VT_UI2:
      switch (vt) {
        case VT_I2:
          pvargDest->iVal = (int16_t) pvarSrc->uiVal;
          break;
        case VT_I4:
          pvargDest->lVal = (int32_t) pvarSrc->uiVal;
          break;
        case VT_R4:
          pvargDest->fltVal = (float) pvarSrc->uiVal;
          break;
        case VT_R8:
          pvargDest->dblVal = (double) pvarSrc->uiVal;
          break;
        case VT_CY:
          pvargDest->cyVal.int64 = (int64_t) pvarSrc->uiVal;
          break;
        case VT_DATE:
          pvargDest->date = (DATE) pvarSrc->uiVal;
          break;
        case VT_BSTR:
          hr = Variant2Bstr(&pvargDest->bstrVal, pvarSrc);
          break;
        case VT_BOOL:
          pvargDest->boolVal = (pvarSrc->uiVal ? VARIANT_TRUE : VARIANT_FALSE);
          break;
        case VT_UI1:
          if (256 <= pvarSrc->uiVal) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->bVal = (uint8_t) pvarSrc->uiVal;
          break;
        case VT_UI4:
          pvargDest->ulVal = pvarSrc->uiVal;
          break;
      }
      break;
    case VT_UI4:
      switch (vt) {
        case VT_I2:
          if (32768 <= pvarSrc->ulVal) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->iVal = (int16_t) pvarSrc->ulVal;
          break;
        case VT_I4:
          pvargDest->lVal = (int32_t) pvarSrc->ulVal;
          break;
        case VT_R4:
          pvargDest->fltVal = (float) pvarSrc->ulVal;
          break;
        case VT_R8:
          pvargDest->dblVal = (double) pvarSrc->ulVal;
          break;
        case VT_CY:
          pvargDest->cyVal.int64 = (int64_t) pvarSrc->ulVal;
          break;
        case VT_DATE:
          pvargDest->date = (DATE) pvarSrc->ulVal;
          break;
        case VT_BSTR:
          hr = Variant2Bstr(&pvargDest->bstrVal, pvarSrc);
          break;
        case VT_BOOL:
          pvargDest->boolVal = (pvarSrc->ulVal ? VARIANT_TRUE : VARIANT_FALSE);
          break;
        case VT_UI1:
          if (256 <= pvarSrc->ulVal) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->bVal = (uint8_t) pvarSrc->ulVal;
          break;
        case VT_UI2:
          if (65536 <= pvarSrc->ulVal) {
            return DISP_E_OVERFLOW;
          }
          pvargDest->uiVal = (uint16_t) pvarSrc->ulVal;
          break;
      }
      break;
    default:
      return DISP_E_BADVARTYPE;
  }

  if (SUCCEEDED(hr)) {
    pvargDest->vt = vt;
  }

  return hr;
}
#endif /* _DN_USE_VARIANT_API */
#endif /* _OLEAUTO_H_ */

#if (_DN_USE_VARIANT_API)
/**
 * @fn         uint32_t ChangeVarType(VARIANT varSrc, uint16_t vt, void *pDest, uint32_t dwSize)
 * @brief      Changes the variant to destination value with the indicated type.
 * @param[in]  varSrc The source variant.
 * @param[in]  vt The variant type.
 * @param[out] pDest The pointer of the destination value.
 * @param[in]  dwSize The maximum number of changed values.
 */
uint32_t
ChangeVarType(VARIANT varSrc, uint16_t vt, void *pDest, uint32_t dwSize)
{
  HRESULT hr = S_OK;
  uint32_t dwRet, dwCnt = 0;
  VARIANT vntTmp;

  if ((pDest == NULL) || (dwSize == 0)) {
    return 0;
  }

  VariantInit(&vntTmp);

  if (varSrc.vt & VT_ARRAY) {
    int32_t i, lEleSize, lMax =
        (int32_t) varSrc.parray->rgsabound[0].cElements;
    void *pVal, *pPos = pDest;

    switch (vt) {
      case VT_UI1:
        lEleSize = 1;
        break;
      case VT_I2:
      case VT_UI2:
      case VT_BOOL:
        lEleSize = 2;
        break;
      case VT_I4:
      case VT_UI4:
      case VT_R4:
        lEleSize = 4;
        break;
      case VT_R8:
      case VT_CY:
        lEleSize = 8;
        break;
      case VT_DATE:
        lEleSize = sizeof(DATE);
        break;
      case VT_BSTR:
        lEleSize = sizeof(BSTR);
        break;
      case VT_VARIANT:
        lEleSize = sizeof(VARIANT);
        break;
      default:
        dwCnt = 0;
        goto exit_proc;
    }

    SafeArrayAccessData(varSrc.parray, &pVal);
    for (i = 0; i < lMax; i++) {
      VariantClear(&vntTmp);
      switch (varSrc.vt ^ VT_ARRAY) {
        case VT_I2:
          vntTmp.vt = VT_I2;
          vntTmp.iVal = *((int16_t *) pVal + i);
          break;
        case VT_I4:
          vntTmp.vt = VT_I4;
          vntTmp.lVal = *((int32_t *) pVal + i);
          break;
        case VT_R4:
          vntTmp.vt = VT_R4;
          vntTmp.fltVal = *((float *) pVal + i);
          break;
        case VT_R8:
          vntTmp.vt = VT_R8;
          vntTmp.dblVal = *((double *) pVal + i);
          break;
        case VT_CY:
          vntTmp.vt = VT_CY;
          vntTmp.cyVal = *((CY *) pVal + i);
          break;
        case VT_DATE:
          vntTmp.vt = VT_DATE;
          vntTmp.date = *((DATE *) pVal + i);
          break;
        case VT_BSTR:
          vntTmp.vt = VT_BSTR;
          vntTmp.bstrVal = SysAllocString(*((BSTR *) pVal + i));
          break;
        case VT_BOOL:
          vntTmp.vt = VT_BOOL;
          vntTmp.boolVal = *((VARIANT_BOOL *) pVal + i);
          break;
        case VT_VARIANT:
          VariantCopy(&vntTmp, (VARIANT *) pVal + i);
          break;
        case VT_UI1:
          vntTmp.vt = VT_UI1;
          vntTmp.bVal = *((uint8_t *) pVal + i);
          break;
        case VT_UI2:
          vntTmp.vt = VT_UI2;
          vntTmp.uiVal = *((uint16_t *) pVal + i);
          break;
        case VT_UI4:
          vntTmp.vt = VT_UI4;
          vntTmp.ulVal = *((uint32_t *) pVal + i);
          break;
        default:
          hr = E_INVALIDARG;
          break;
      }

      if (FAILED(hr)) {
        break;
      }

      if (vt != VT_VARIANT) {
        if (vntTmp.vt & VT_ARRAY) {
          break;
        }

        dwRet = ChangeVarType(vntTmp, vt, pPos, 1);
        if (dwRet == 0) {
          break;
        }
      } else {
        hr = VariantCopy((VARIANT*) pPos, &vntTmp);
        if (FAILED(hr)) {
          break;
        }
        dwRet = 1;
      }

      dwCnt += dwRet;
      pPos = ((char *) pPos + lEleSize);

      if (dwCnt >= dwSize) {
        break;
      }
    }
    SafeArrayUnaccessData(varSrc.parray);
  } else {
    VariantCopy(&vntTmp, &varSrc);
    if (vt != VT_VARIANT) {
      hr = VariantChangeType(&vntTmp, &vntTmp, 0, vt);
      if (FAILED(hr)) {
        dwCnt = 0;
        goto exit_proc;
      }
    }

    dwCnt = 1;
    switch (vt) {
      case VT_I2:
        *(int16_t *) pDest = vntTmp.iVal;
        break;
      case VT_I4:
        *(int32_t *) pDest = vntTmp.lVal;
        break;
      case VT_R4:
        *(float *) pDest = vntTmp.fltVal;
        break;
      case VT_R8:
        *(double *) pDest = vntTmp.dblVal;
        break;
      case VT_CY:
        *(CY *) pDest = vntTmp.cyVal;
        break;
      case VT_DATE:
        *(DATE *) pDest = vntTmp.date;
        break;
      case VT_BSTR:
        *(BSTR *) pDest = SysAllocString(vntTmp.bstrVal);
        break;
      case VT_BOOL:
        *(VARIANT_BOOL *) pDest = vntTmp.boolVal ? VARIANT_TRUE : VARIANT_FALSE;
        break;
      case VT_VARIANT:
        VariantCopy((VARIANT *) pDest, &vntTmp);
        break;
      case VT_UI1:
        *(uint8_t *) pDest = vntTmp.bVal;
        break;
      case VT_UI2:
        *(uint16_t *) pDest = vntTmp.uiVal;
        break;
      case VT_UI4:
        *(uint32_t *) pDest = vntTmp.ulVal;
        break;
      default:
        dwCnt = 0;
        break;
    }
  }

exit_proc:
  VariantClear(&vntTmp);

  return dwCnt;
}

/**
 * @fn         HRESULT GetOptionValue(BSTR bstrSrc, BSTR bstrKey, uint16_t vt, VARIANT *pvarDest)
 * @brief      Searchs the key string from source string and sets the value to the destination variant with the indicated type.
 * @param[in]  bstrSrc The source string.
 * @param[in]  bstrKey The key string.
 * @param[in]  vt The variant type.
 * @param[out] pvarDest The destination variant.
 */
HRESULT
GetOptionValue(BSTR bstrSrc, BSTR bstrKey, uint16_t vt, VARIANT *pvarDest)
{
  HRESULT hr = S_OK;
  int iLevelCount = 0, bGetTitle = 0;
  long lTitleTerm;
  uint16_t i, uiLenTarget = 0, uiLenOptTitle = 0, uiLenOneOpt = 0;
  wchar_t *wchTmp, *wchPos, wchStart = -1, wchEnd = -1;
  BSTR bstrTmp, bstrTarget, bstrOptTitle, bstrOneOpt;
  VARIANT vntTmp;

  if ((bstrKey == NULL) || (pvarDest == NULL)) {
    return E_INVALIDARG;
  }

  if (bstrSrc != NULL) {
    bstrTarget = SysAllocString(bstrSrc);
  } else {
    bstrTarget = SysAllocString(L"");
  }

  bstrOptTitle = SysAllocString(L"");
  bstrOneOpt = SysAllocString(L"");

  uiLenTarget = SysStringLen(bstrTarget);
  wchTmp = bstrTarget;

  for (i = 0; i <= uiLenTarget; i++, wchTmp++) {
    /* Sets the beginning and ending character for nest */
    if (!iLevelCount) {
      switch (*wchTmp) {
        case L'(':
        case L')':
          wchStart = L'(';
          wchEnd = L')';
          break;
        case L'[':
        case L']':
          wchStart = L'[';
          wchEnd = L']';
          break;
        case L'{':
        case L'}':
          wchStart = L'{';
          wchEnd = L'}';
          break;
        case L'<':
        case L'>':
          wchStart = L'<';
          wchEnd = L'>';
          break;
      }
    }

    /* Sets the nest level */
    if (*wchTmp == wchStart) {
      iLevelCount++;
      if (iLevelCount == 1) {
        memcpy(wchTmp, wchTmp + 1, sizeof(wchar_t) * (uiLenTarget - i));
        uiLenTarget--;
        i--;
        wchTmp--;
        continue;
      }
    }
    else if (*wchTmp == wchEnd) {
      iLevelCount--;

      if (iLevelCount < 0) {
        hr = E_FAIL;
        goto exit_proc;
      }

      if (!iLevelCount) {
        memcpy(wchTmp, wchTmp + 1, sizeof(wchar_t) * (uiLenTarget - i));
        uiLenTarget--;
        i--;
        wchTmp--;
        continue;
      }
    }

    /* Gets the option name */
    if (!iLevelCount) {
      if (*wchTmp == L'=') {
        if (bGetTitle) {
          hr = E_FAIL;
          goto exit_proc;
        }

        if (uiLenOptTitle > 0) {
          SysFreeString(bstrOptTitle);
          bstrOptTitle = SysAllocStringLen(wchTmp - uiLenOptTitle,
              uiLenOptTitle);
          uiLenOptTitle = 0;
        }

        bGetTitle = 1;
        continue;
      }

      if (*wchTmp == L',' || *wchTmp == L'\0') {
        if (uiLenOneOpt > 0) {
          SysFreeString(bstrOneOpt);
          bstrOneOpt = SysAllocStringLen(wchTmp - uiLenOneOpt,
              uiLenOneOpt);
          uiLenOneOpt = 0;
        }

        bstrTmp = bstrOptTitle;
        wchPos = bstrTmp;
        while (wchPos[0] == L' ') {
          wchPos++;
        }

        lTitleTerm = wcslen(wchPos) - 1;
        while (wchPos[lTitleTerm] == L' ') {
          wchPos[lTitleTerm] = L'\0';
          lTitleTerm--;
        }

        bstrOptTitle = SysAllocString(wchPos);
        SysFreeString(bstrTmp);

        if (!_wcsicmp(bstrOptTitle, bstrKey)) {
          if (!SysStringLen(bstrOneOpt)) {
            if (vt == VT_BOOL) {
              pvarDest->vt = VT_BOOL;
              pvarDest->boolVal = VARIANT_TRUE;
            } else {
              VariantClear(pvarDest);
            }
            hr = S_OK;
            goto exit_proc;
          }
          break;
        }

        if (*wchTmp == L',') {
          SysFreeString(bstrOptTitle);
          bstrOptTitle = SysAllocString(L"");
          uiLenOptTitle = 0;

          SysFreeString(bstrOneOpt);
          bstrOneOpt = SysAllocString(L"");
          uiLenOneOpt = 0;

          bGetTitle = 0;
          continue;
        } else {
          VariantClear(pvarDest);
          hr = S_OK;
          goto exit_proc;
        }
      }
    }

    if (bGetTitle) {
      uiLenOneOpt++;
    } else {
      uiLenOptTitle++;
    }
  }

  if (iLevelCount != 0) {
    hr = E_FAIL;
    goto exit_proc;
  }

  bstrTmp = bstrOneOpt;
  wchPos = bstrTmp;
  while (wchPos[0] == L' ') {
    wchPos++;
  }

  lTitleTerm = wcslen(wchPos) - 1;
  while (wchPos[lTitleTerm] == L' ') {
    wchPos[lTitleTerm] = L'\0';
    lTitleTerm--;
  }

  bstrOneOpt = SysAllocString(wchPos);
  SysFreeString(bstrTmp);

  vntTmp.vt = VT_BSTR;
  vntTmp.bstrVal = bstrOneOpt;

  hr = VariantChangeType(pvarDest, &vntTmp, 0, vt);

  exit_proc: SysFreeString(bstrTarget);
  SysFreeString(bstrOptTitle);
  SysFreeString(bstrOneOpt);

  return hr;
}
#endif /* _DN_USE_VARIANT_API */
