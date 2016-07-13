//
//  ViewController.m
//  NHOpenCVZBarPro
//
//  Created by hu jiaju on 15/10/23.
//  Copyright © 2015年 hu jiaju. All rights reserved.
//

#import "ViewController.h"
#import "NHScanVCR.h"

@interface ViewController ()

@end

@implementation ViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view, typically from a nib.
    
    self.title = @"OpenCV&ZBar";
    
    CGRect infoRect = CGRectMake(100, 100, 200, 50);
    UIButton *btn = [UIButton buttonWithType:UIButtonTypeCustom];
    btn.frame = infoRect;
    [btn setTitle:@"Scan Qr" forState:UIControlStateNormal];
    [btn setTitleColor:[UIColor blackColor] forState:UIControlStateNormal];
    [btn addTarget:self action:@selector(scanQrCode) forControlEvents:UIControlEventTouchUpInside];
    [self.view addSubview:btn];
    
    infoRect.origin.y += 100;
    btn = [UIButton buttonWithType:UIButtonTypeCustom];
    btn.frame = infoRect;
    [btn setTitle:@"Scan ID" forState:UIControlStateNormal];
    [btn setTitleColor:[UIColor blackColor] forState:UIControlStateNormal];
    [btn addTarget:self action:@selector(scanIDCard) forControlEvents:UIControlEventTouchUpInside];
    [self.view addSubview:btn];
    
    infoRect.origin.y += 100;
    btn = [UIButton buttonWithType:UIButtonTypeCustom];
    btn.frame = infoRect;
    [btn setTitle:@"Scan Bank" forState:UIControlStateNormal];
    [btn setTitleColor:[UIColor blackColor] forState:UIControlStateNormal];
    [btn addTarget:self action:@selector(scanBankCard) forControlEvents:UIControlEventTouchUpInside];
    [self.view addSubview:btn];
}

- (void)scanQrCode {
    NHScanVCR *scanView = [[NHScanVCR alloc] initWithScanType:NHScanTypeQR];
    [self.navigationController pushViewController:scanView animated:true];
}

- (void)scanIDCard {
    NHScanVCR *scanView = [[NHScanVCR alloc] initWithScanType:NHScanTypeID];
    [self.navigationController pushViewController:scanView animated:true];
}

- (void)scanBankCard {
    NHScanVCR *scanView = [[NHScanVCR alloc] initWithScanType:NHScanTypeBank];
    [self.navigationController pushViewController:scanView animated:true];
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

@end
