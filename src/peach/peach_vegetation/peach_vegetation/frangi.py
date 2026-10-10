"""2D Frangi vesselness on PyTorch (GPU or CPU). Zero ROS."""

from __future__ import annotations

from typing import Sequence

import numpy as np


def resolve_device(requested: str) -> str:
    """
    Map ``auto`` / ``cuda`` / ``cuda:0`` / ``cpu`` to a torch device string.

    ``auto`` falls back to CPU when CUDA is missing. Explicit CUDA without
    a device raises RuntimeError so configure fails loudly.
    """
    try:
        import torch
    except ImportError as exc:
        if requested in ('auto', 'cpu'):
            raise RuntimeError('torch 未安装') from exc
        raise RuntimeError('torch 未安装，无法使用 CUDA') from exc
    if requested == 'cpu':
        return 'cpu'
    cuda_ok = torch.cuda.is_available()
    if requested == 'auto':
        return 'cuda:0' if cuda_ok else 'cpu'
    if requested in ('cuda', 'cuda:0'):
        if not cuda_ok:
            raise RuntimeError('请求 CUDA 但 torch.cuda.is_available() 为假')
        return 'cuda:0'
    raise RuntimeError(f'未知 device={requested}')


def _gaussian_kernel1d(sigma: float, device, dtype):
    """Return a 1-D Gaussian kernel spanning about 3 sigma."""
    import torch
    radius = max(1, int(round(3.0 * float(sigma))))
    x = torch.arange(-radius, radius + 1, device=device, dtype=dtype)
    kernel = torch.exp(-(x * x) / (2.0 * float(sigma) * float(sigma)))
    kernel = kernel / kernel.sum()
    return kernel, radius


def _gauss_blur(image, sigma: float):
    """Separable replicate-pad Gaussian blur on NCHW tensor."""
    import torch.nn.functional as F
    kernel, radius = _gaussian_kernel1d(sigma, image.device, image.dtype)
    kx = kernel.view(1, 1, 1, -1)
    ky = kernel.view(1, 1, -1, 1)
    padded = F.pad(image, (radius, radius, 0, 0), mode='replicate')
    blurred = F.conv2d(padded, kx)
    padded = F.pad(blurred, (0, 0, radius, radius), mode='replicate')
    return F.conv2d(padded, ky)


def _central_diff_x(image):
    """d/dx with replicate pad, kernel [-0.5, 0, 0.5]."""
    import torch
    import torch.nn.functional as F
    kernel = torch.tensor(
        [[[[-0.5, 0.0, 0.5]]]], device=image.device, dtype=image.dtype)
    padded = F.pad(image, (1, 1, 0, 0), mode='replicate')
    return F.conv2d(padded, kernel)


def _central_diff_y(image):
    """d/dy with replicate pad, kernel [-0.5, 0, 0.5]^T."""
    import torch
    import torch.nn.functional as F
    kernel = torch.tensor(
        [[[[-0.5], [0.0], [0.5]]]], device=image.device, dtype=image.dtype)
    padded = F.pad(image, (0, 0, 1, 1), mode='replicate')
    return F.conv2d(padded, kernel)


def _frangi_at_sigma(gray, sigma: float, beta: float, gamma: float):
    """
    Scale-normalized 2-D Frangi response at one sigma.

    ``gray`` is NCHW float, already inverted if looking for dark ridges.
    """
    import torch
    blurred = _gauss_blur(gray, sigma)
    ix = _central_diff_x(blurred)
    iy = _central_diff_y(blurred)
    ixx = _central_diff_x(ix) * (sigma * sigma)
    ixy = _central_diff_y(ix) * (sigma * sigma)
    iyy = _central_diff_y(iy) * (sigma * sigma)
    tmp = torch.sqrt((ixx - iyy) * (ixx - iyy) + 4.0 * ixy * ixy)
    mu = 0.5 * (ixx + iyy)
    lam_a = mu + 0.5 * tmp
    lam_b = mu - 0.5 * tmp
    # Sort by absolute value: lam1 smaller |λ|, lam2 larger |λ|.
    swap = torch.abs(lam_a) > torch.abs(lam_b)
    lam1 = torch.where(swap, lam_b, lam_a)
    lam2 = torch.where(swap, lam_a, lam_b)
    eps = torch.tensor(1e-10, device=gray.device, dtype=gray.dtype)
    rb = torch.abs(lam1) / torch.clamp(torch.abs(lam2), min=eps)
    struct = torch.sqrt(lam1 * lam1 + lam2 * lam2)
    vessel = torch.exp(-(rb * rb) / (2.0 * beta * beta))
    vessel = vessel * (1.0 - torch.exp(-(struct * struct) / (2.0 * gamma * gamma)))
    # After inversion, bright ridges have λ2 < 0 (largest-|λ| negative).
    vessel = torch.where(lam2 < 0, vessel, torch.zeros_like(vessel))
    return vessel


def frangi_vesselness(
        gray: np.ndarray,
        *,
        sigmas: Sequence[float] = (1, 3, 5, 7, 9),
        beta: float = 0.5,
        gamma: float = 15.0,
        black_ridges: bool = True,
        device: str = 'cpu') -> np.ndarray:
    """
    Return HxW float32 2-D Frangi vesselness (host numpy).

    ``gray`` is HxW float or uint8 (uint8 scaled to [0, 1]). ``sigmas`` are
    Gaussian scales in pixels. ``black_ridges`` selects dark lines on a
    lighter background.
    """
    import torch
    if gray.ndim != 2:
        raise ValueError('frangi_vesselness 需要 HxW 灰度图')
    image = np.asarray(gray)
    if image.dtype == np.uint8:
        image = image.astype(np.float32) / 255.0
    else:
        image = image.astype(np.float32)
    if black_ridges:
        image = 1.0 - image
    tensor = torch.from_numpy(image).to(device=device, dtype=torch.float32)
    tensor = tensor.unsqueeze(0).unsqueeze(0)
    acc = torch.zeros_like(tensor)
    for sigma in sigmas:
        acc = torch.maximum(acc, _frangi_at_sigma(tensor, float(sigma), beta, gamma))
    return acc.squeeze(0).squeeze(0).detach().cpu().numpy()


def threshold_ridges(
        vesselness: np.ndarray,
        gray_u8: np.ndarray,
        *,
        percentile: float,
        dark_max: float,
        dilate_px: int) -> np.ndarray:
    """
    Return a bool ridge mask from percentile-thresholded Frangi.

    Keeps pixels whose luma is at most ``dark_max`` and optionally dilates.
    """
    if vesselness.size == 0:
        return np.zeros(vesselness.shape, dtype=bool)
    finite = np.isfinite(vesselness)
    if not np.any(finite):
        return np.zeros(vesselness.shape, dtype=bool)
    cut = float(np.percentile(vesselness[finite], percentile))
    mask = vesselness >= cut
    mask &= gray_u8 <= dark_max
    if dilate_px > 0:
        mask = _dilate_bool(mask, dilate_px)
        mask &= gray_u8 <= dark_max
    return mask


def _dilate_bool(mask: np.ndarray, radius: int) -> np.ndarray:
    """方形态学膨胀（cv2；cv_bridge 硬依赖保证可用）."""
    import cv2
    kernel = np.ones((2 * radius + 1, 2 * radius + 1), np.uint8)
    return cv2.dilate(
        mask.astype(np.uint8), kernel, iterations=1).astype(bool)
