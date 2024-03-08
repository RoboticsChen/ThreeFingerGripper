#include "sys.h"

// THUMBָ�֧�ֻ������
// �������·���ʵ��ִ�л��ָ��WFI
void WFI_SET(void) { __ASM volatile("WFI"); }

// �ر������ж�(���ǲ�����fault��NMI�ж�)
void INTX_DISABLE(void) {
  __ASM volatile("CPSID   I");
  __ASM volatile("BX      LR");
}

// ���������ж�
void INTX_ENABLE(void) {
  __ASM volatile("CPSIE   I");
  __ASM volatile("BX      LR");
}

// ����ջ����ַ
// addr:ջ����ַ
void MSR_MSP(u32 addr) {
  __ASM volatile("MSR MSP, r0");  // set Main Stack value
  __ASM volatile("BX LR");
}
