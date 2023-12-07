# Copyright (c) OpenMMLab. All rights reserved.
from typing import Optional

import torch
import torch.nn as nn
from torch import Tensor
import numpy as np

from mmdet3d.registry import MODELS
from mmdet.models.losses.utils import weighted_loss


@weighted_loss
def neg_log_pdf_loss(pred: Tensor, target: Tensor) -> Tensor:
    """
        -Log(PDF) loss
    """
    # EXPLICITLY DEFINED -log(PDF)
    # sigma = torch.abs(pred[:, 7:])

    # COMPUTE ELU FOR SIGMA (prefferred method via towards data science article)
    elu = torch.nn.ELU(alpha=1.0, inplace=False)
    sigma = elu(pred[:, 7:]) + 1
    # enforce sigma has no negative values or nan rows
    sigma[sigma < 0] = 0.0
    sigma = sigma[~torch.any(sigma.isnan(), dim=1)]
    # get bbox pred and remove any nan rows
    bbox = pred[:, :7]
    bbox = bbox[~torch.any(bbox.isnan(), dim=1)]


    # README: 
    # explicit computation of PDF was not working, having errors when computing gradient.
    # if torch.isnan(sigma).any():
    #     print("SIGMA NANNED OUT!")

    # const = 1/(np.sqrt(2*(np.pi))*sigma)

    # if torch.isnan((pred)).any():
    #     print("PRED NAN!!")

    # PDF = const * torch.exp(-((target - bbox)**2 / 2*(sigma**2)))

    # if torch.isnan(torch.isnan(PDF)).any():
    #     print("PDF IS NAN")

    # loss = -torch.log(PDF)

    # if torch.isnan(torch.isnan(loss)).any():
    #     print("LOSS IS NAN")
    # # print(loss)

    # CREATE NORMAL DISTROBUTION BASED ON bbox pred & sigma!
    # pytorch distributions from the article, needs the fc input (mu & sigma)
    dist = torch.distributions.Normal(loc=bbox, scale=sigma)
    loss = torch.nanmean(-dist.log_prob(target)) # nan mean sums all non-nan values

    return loss

@weighted_loss
def smooth_l1_loss(pred: Tensor, target: Tensor, beta: float = 1.0) -> Tensor:
    """Smooth L1 loss.

    Args:
        pred (Tensor): The prediction.
        target (Tensor): The learning target of the prediction.
        beta (float, optional): The threshold in the piecewise function.
            Defaults to 1.0.

    Returns:
        Tensor: Calculated loss
    """
    assert beta > 0
    if target.numel() == 0:
        return pred.sum() * 0

    assert pred.size() == target.size()
    diff = torch.abs(pred - target)
    loss = torch.where(diff < beta, 0.5 * diff * diff / beta,
                       diff - 0.5 * beta)
    return loss


@weighted_loss
def l1_loss(pred: Tensor, target: Tensor) -> Tensor:
    """L1 loss.

    Args:
        pred (Tensor): The prediction.
        target (Tensor): The learning target of the prediction.

    Returns:
        Tensor: Calculated loss
    """
    if target.numel() == 0:
        return pred.sum() * 0

    assert pred.size() == target.size()
    loss = torch.abs(pred - target)
    return loss


@MODELS.register_module()
class SmoothL1Loss(nn.Module):
    """Smooth L1 loss.

    Args:
        beta (float, optional): The threshold in the piecewise function.
            Defaults to 1.0.
        reduction (str, optional): The method to reduce the loss.
            Options are "none", "mean" and "sum". Defaults to "mean".
        loss_weight (float, optional): The weight of loss.
    """

    def __init__(self,
                 beta: float = 1.0,
                 reduction: str = 'mean',
                 loss_weight: float = 1.0) -> None:
        super().__init__()
        self.beta = beta
        self.reduction = reduction
        self.loss_weight = loss_weight

    def forward(self,
                pred: Tensor,
                target: Tensor,
                weight: Optional[Tensor] = None,
                avg_factor: Optional[int] = None,
                reduction_override: Optional[str] = None,
                **kwargs) -> Tensor:
        """Forward function.

        Args:
            pred (Tensor): The prediction.
            target (Tensor): The learning target of the prediction.
            weight (Tensor, optional): The weight of loss for each
                prediction. Defaults to None.
            avg_factor (int, optional): Average factor that is used to average
                the loss. Defaults to None.
            reduction_override (str, optional): The reduction method used to
                override the original reduction method of the loss.
                Defaults to None.

        Returns:
            Tensor: Calculated loss
        """
        if weight is not None and not torch.any(weight > 0):
            if pred.dim() == weight.dim() + 1:
                weight = weight.unsqueeze(1)
            return (pred * weight).sum()
        assert reduction_override in (None, 'none', 'mean', 'sum')
        reduction = (
            reduction_override if reduction_override else self.reduction)
        loss_bbox = self.loss_weight * smooth_l1_loss(
            pred,
            target,
            weight,
            beta=self.beta,
            reduction=reduction,
            avg_factor=avg_factor,
            **kwargs)
        return loss_bbox


@MODELS.register_module()
class L1Loss(nn.Module):
    """L1 loss.

    Args:
        reduction (str, optional): The method to reduce the loss.
            Options are "none", "mean" and "sum".
        loss_weight (float, optional): The weight of loss.
    """

    def __init__(self,
                 reduction: str = 'mean',
                 loss_weight: float = 1.0) -> None:
        super().__init__()
        self.reduction = reduction
        self.loss_weight = loss_weight

    def forward(self,
                pred: Tensor,
                target: Tensor,
                weight: Optional[Tensor] = None,
                avg_factor: Optional[int] = None,
                reduction_override: Optional[str] = None) -> Tensor:
        """Forward function.

        Args:
            pred (Tensor): The prediction.
            target (Tensor): The learning target of the prediction.
            weight (Tensor, optional): The weight of loss for each
                prediction. Defaults to None.
            avg_factor (int, optional): Average factor that is used to average
                the loss. Defaults to None.
            reduction_override (str, optional): The reduction method used to
                override the original reduction method of the loss.
                Defaults to None.

        Returns:
            Tensor: Calculated loss
        """
        if weight is not None and not torch.any(weight > 0):
            if pred.dim() == weight.dim() + 1:
                weight = weight.unsqueeze(1)
            return (pred * weight).sum()
        assert reduction_override in (None, 'none', 'mean', 'sum')
        reduction = (
            reduction_override if reduction_override else self.reduction)
        loss_bbox = self.loss_weight * l1_loss(
            pred, target, weight, reduction=reduction, avg_factor=avg_factor)
        return loss_bbox
    
@MODELS.register_module()
class NegativeLogPDFLoss(nn.Module):
    """Negative Log(PDF) loss.

    Args:
        reduction (str, optional): The method to reduce the loss.
            Options are "none", "mean" and "sum".
        loss_weight (float, optional): The weight of loss.
    """

    def __init__(self,
                 reduction: str = 'mean',
                 loss_weight: float = 1.0) -> None:
        super().__init__()
        self.reduction = reduction
        self.loss_weight = loss_weight

    def forward(self,
                pred: Tensor,
                target: Tensor,
                weight: Optional[Tensor] = None,
                avg_factor: Optional[int] = None,
                reduction_override: Optional[str] = None) -> Tensor:
        """Forward function.

        Args:
            pred (Tensor): The prediction.
            target (Tensor): The learning target of the prediction.
            weight (Tensor, optional): The weight of loss for each
                prediction. Defaults to None.
            avg_factor (int, optional): Average factor that is used to average
                the loss. Defaults to None.
            reduction_override (str, optional): The reduction method used to
                override the original reduction method of the loss.
                Defaults to None.

        Returns:
            Tensor: Calculated loss
        """
        # if weight is not None and not torch.any(weight > 0):
        #     if pred.dim() == weight.dim() + 1:
        #         weight = weight.unsqueeze(1)
        #     return (pred * weight).sum()
        # assert reduction_override in (None, 'none', 'mean', 'sum')
        # reduction = (
        #     reduction_override if reduction_override else self.reduction)
        loss_bbox = self.loss_weight * neg_log_pdf_loss(pred, target)
        return loss_bbox